from __future__ import annotations
from dataclasses import dataclass
from scenario import ActionType, BATTERY_MAS, CHARGE_MA, DRIVE_MA, ExecutedAction, Knowledge, PlannedAction, Scenario, VehicleSnapshot, VehicleState, initial_states, outage_at

@dataclass
class RuntimeAction:
    planned: PlannedAction
    start: int
    end: int
    soc_start: int

@dataclass(frozen=True)
class Result:
    controller: str
    completed: bool
    makespan: int
    total_waiting_time: int
    transport_time_fraction: float
    charger_utilization: float
    charging_visits: int
    actions: tuple[ExecutedAction, ...]
    failure_reason: str | None

class Simulator:
    def __init__(self, scenario: Scenario, controller, verbose=False):
        self.scenario = scenario
        self.controller = controller
        self.verbose = verbose
        self.time = 0
        self.vehicles: dict[int, VehicleState] = initial_states(scenario)
        self.active: dict[int, RuntimeAction] = {}
        self.pending: dict[int, PlannedAction] = {}
        self.history: list[ExecutedAction] = []
        self._charger_trip_started_at: dict[int, int] = {}
        self.charger_time = 0
        self.charging_visits = 0
        self.failure_reason = None
        self._skip_zero_tasks()

    def _skip_zero_tasks(self):
        for v in self.vehicles.values():
            while v.task_idx < len(v.tasks) and v.tasks[v.task_idx] <= 0:
                v.task_idx += 1

    def snapshots(self, flexible_charges=False):
        snapshots = []
        for v in self.vehicles.values():
            action = self.active.get(v.id)
            if action is None:
                snapshots.append(VehicleSnapshot(
                    v.id, v.soc, v.tasks, v.task_idx, v.location, v.busy_until,
                    charger_trip_started_at=self._charger_trip_started_at.get(v.id),
                ))
                continue
            if flexible_charges and action.planned.kind == ActionType.CHARGE:
                current_soc = action.soc_start + self._delta_soc(ActionType.CHARGE, self.time - action.start)
                snapshots.append(VehicleSnapshot(
                    v.id, current_soc, v.tasks, v.task_idx, v.location, self.time,
                    current_soc, v.task_idx, "charger", charging=True,
                ))
                continue
            duration = action.end - action.start
            ready_soc = action.soc_start + self._delta_soc(action.planned.kind, duration)
            ready_task_idx = v.task_idx + (action.planned.kind == ActionType.DELIVERY)
            ready_location = v.location
            if action.planned.kind == ActionType.DRIVE_TO_CHARGER:
                ready_location = "charger"
            elif action.planned.kind in (ActionType.DRIVE_FROM_CHARGER, ActionType.DELIVERY):
                ready_location = "mission"
            snapshots.append(VehicleSnapshot(
                v.id, v.soc, v.tasks, v.task_idx, v.location, v.busy_until,
                ready_soc, ready_task_idx, ready_location,
                charger_trip_started_at=action.start if action.planned.kind == ActionType.DRIVE_TO_CHARGER else None,
            ))
        return tuple(snapshots)

    def charger_busy_until(self, flexible_charges=False):
        ends = [
            r.end for r in self.active.values()
            if r.planned.kind == ActionType.CHARGE and not flexible_charges
        ]
        return max(ends, default=self.time)

    def knowledge(self):
        observed = tuple(o for o in self.scenario.outages if o.start <= self.time)
        return Knowledge(observed, self.scenario.outages)

    def free_with_work(self):
        return [v for v in self.vehicles.values() if v.busy_until <= self.time and v.task_idx < len(v.tasks)]

    def finished(self):
        return all(v.task_idx >= len(v.tasks) for v in self.vehicles.values()) and not self.active and not self.pending

    def replan(self):
        flexible_charges = self.controller.name in {"OPTIMIZER", "ORACLE"}
        active_charges = any(
            action.planned.kind == ActionType.CHARGE
            for action in self.active.values()
        )
        if self.failure_reason or (not self.free_with_work() and not (flexible_charges and active_charges)):
            return
        actions = self.controller.decide(
            self.time,
            self.snapshots(flexible_charges=flexible_charges),
            self.charger_busy_until(flexible_charges=flexible_charges),
            self.knowledge(),
        )
        self.pending.clear()
        flexible_vehicles = set()
        if flexible_charges:
            for action in actions:
                active = self.active.get(action.vehicle)
                if (
                    active is None
                    or active.planned.kind != ActionType.CHARGE
                    or action.start != self.time
                ):
                    continue
                flexible_vehicles.add(action.vehicle)
                if action.kind == ActionType.CHARGE:
                    active.end = action.end
                    active.planned = action
                    self.vehicles[action.vehicle].busy_until = action.end
                elif action.kind == ActionType.DRIVE_FROM_CHARGER:
                    vehicle = self.vehicles[action.vehicle]
                    self.charger_time += self.time - active.start
                    current_soc = active.soc_start + self._delta_soc(
                        ActionType.CHARGE,
                        self.time - active.start,
                    )
                    vehicle.soc = current_soc
                    if self.time > active.start:
                        completed_charge = ExecutedAction(
                            action.vehicle,
                            ActionType.CHARGE,
                            active.start,
                            self.time,
                            active.soc_start,
                            current_soc,
                            active.planned.task_idx,
                        )
                        self.history.append(completed_charge)
                        on_action_completed = getattr(self.controller, "on_action_completed", None)
                        if on_action_completed is not None:
                            on_action_completed(completed_charge)
                    del self.active[action.vehicle]
                    vehicle.busy_until = self.time
                    self.pending[action.vehicle] = action
        free_ids = {v.id for v in self.free_with_work()}
        for a in actions:
            if a.vehicle not in free_ids or a.vehicle in flexible_vehicles:
                continue
            if a.start != self.time or a.end <= a.start:
                self.failure_reason = f"invalid controller action for V{a.vehicle}: {a}"
                return
            if a.vehicle in self.pending:
                self.failure_reason = f"multiple controller actions for V{a.vehicle}"
                return
            self.pending[a.vehicle] = a
        missing = free_ids - set(self.pending)
        if missing:
            self.failure_reason = "controller returned no feasible action for " + ", ".join(f"V{x}" for x in sorted(missing))

    def _delta_soc(self, kind, duration):
        if kind == ActionType.DELIVERY or kind in (ActionType.DRIVE_TO_CHARGER, ActionType.DRIVE_FROM_CHARGER):
            rate = DRIVE_MA
            return -rate * duration
        if kind == ActionType.CHARGE:
            return CHARGE_MA * duration
        return -2000 * duration

    def start_pending(self):
        changed = False
        for vid, p in list(self.pending.items()):
            if p.start != self.time:
                continue
            v = self.vehicles[vid]
            if v.busy_until > self.time:
                continue
            if p.kind == ActionType.CHARGE:
                outage = outage_at(self.time, self.scenario.outages)
                trip_started = self._charger_trip_started_at.get(vid)
                trip_is_committed = outage is not None and trip_started is not None and outage.start > trip_started
                if (outage is not None and not trip_is_committed) or self.charger_busy_until() > self.time:
                    del self.pending[vid]
                    changed = True
                    continue
            duration = p.end - p.start
            if p.kind == ActionType.DELIVERY and p.task_idx is not None:
                duration = self.scenario.actual_delivery[(vid, p.task_idx)]
            end = self.time + duration
            projected = v.soc + self._delta_soc(p.kind, duration)
            if projected < 0 or projected > BATTERY_MAS:
                self.failure_reason = f"SoC infeasible during {p.kind.value} for V{vid}"
                return changed
            self.active[vid] = RuntimeAction(p, self.time, end, v.soc)
            if p.kind == ActionType.DRIVE_TO_CHARGER:
                self._charger_trip_started_at[vid] = self.time
            elif p.kind in (ActionType.CHARGE, ActionType.DRIVE_FROM_CHARGER):
                self._charger_trip_started_at.pop(vid, None)
            on_action_started = getattr(self.controller, "on_action_started", None)
            if on_action_started is not None:
                on_action_started(p)
            v.busy_until = end
            if p.kind == ActionType.CHARGE:
                self.charging_visits += 1
            del self.pending[vid]
            changed = True
        return changed

    def finish_actions(self):
        changed = False
        for vid, r in list(self.active.items()):
            if r.end != self.time:
                continue
            v = self.vehicles[vid]
            duration = r.end - r.start
            v.soc += self._delta_soc(r.planned.kind, duration)
            if v.soc < 0 or v.soc > BATTERY_MAS:
                self.failure_reason = f"SoC out of bounds after {r.planned.kind.value} for V{vid}"
                return changed
            if r.planned.kind == ActionType.DRIVE_TO_CHARGER:
                v.location = "charger"
            elif r.planned.kind == ActionType.DRIVE_FROM_CHARGER:
                v.location = "mission"
            elif r.planned.kind == ActionType.DELIVERY:
                v.location = "mission"
                v.task_idx += 1
                v.delivery_time += duration
                v.completion_times.append(self.time)
                while v.task_idx < len(v.tasks) and v.tasks[v.task_idx] <= 0:
                    v.task_idx += 1
            elif r.planned.kind == ActionType.CHARGE:
                self.charger_time += duration
            self.history.append(ExecutedAction(vid, r.planned.kind, r.start, r.end, r.soc_start, v.soc, r.planned.task_idx))
            on_action_completed = getattr(self.controller, "on_action_completed", None)
            if on_action_completed is not None:
                on_action_completed(self.history[-1])
            v.busy_until = 0
            del self.active[vid]
            changed = True
        return changed

    def next_event(self):
        candidates = [r.end for r in self.active.values() if r.end > self.time]
        candidates += [p.start for p in self.pending.values() if p.start > self.time]
        for o in self.scenario.outages:
            if o.start > self.time:
                candidates.append(o.start)
            if o.end > self.time:
                candidates.append(o.end)
        return min(candidates) if candidates else None

    def run(self):
        self.replan()
        events = 0
        last_time = -1
        while not self.failure_reason and not self.finished():
            events += 1
            if events > 100000:
                self.failure_reason = "event-loop guard: more than 100000 events"
                break
            if self.time < last_time:
                self.failure_reason = "event-loop guard: simulation time moved backwards"
                break
            last_time = self.time
            started = self.start_pending()
            if self.failure_reason or self.finished():
                break
            if started:
                # A rejected charge may have been removed above; immediately ask the
                # controller for a replacement action at the same simulation time.
                missing = [v for v in self.free_with_work() if v.id not in self.pending and v.id not in self.active]
                if missing:
                    self.replan()
                    if self.failure_reason:
                        break
                    self.start_pending()
                    if self.failure_reason or self.finished():
                        break
            t = self.next_event()
            if t is None:
                self.failure_reason = "simulation reached a state with unfinished work but no future event"
                break
            self.time = t
            ended = self.finish_actions()
            if self.failure_reason:
                break
            outage_event = any(o.start == self.time or o.end == self.time for o in self.scenario.outages)
            if ended or outage_event:
                self._skip_zero_tasks()
                self.replan()
                if self.failure_reason:
                    break
                if self.start_pending():
                    missing = [v for v in self.free_with_work() if v.id not in self.pending and v.id not in self.active]
                    if missing:
                        self.replan()
                        if self.failure_reason:
                            break
                        self.start_pending()
        completed = self.failure_reason is None and self.finished()
        makespan = self.time
        total_waiting_time = sum(a.end - a.start for a in self.history if a.kind == ActionType.WAIT)
        total_delivery = sum(v.delivery_time for v in self.vehicles.values())
        transport_fraction = total_delivery / (len(self.vehicles) * makespan) if makespan else 0.0
        utilization = self.charger_time / makespan if makespan else 0.0
        return Result(self.controller.name, completed, makespan, total_waiting_time, transport_fraction, utilization, self.charging_visits, tuple(self.history), self.failure_reason)
