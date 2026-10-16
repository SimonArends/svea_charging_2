from __future__ import annotations
from dataclasses import replace
from scenario import ActionType, BATTERY_MAS, CHARGE_MA, CHARGE_TARGET_MAS, DEFAULT_INITIAL_SOCS, DEFAULT_TASKS, DRIVE_MA, FROM_CHARGER_S, Knowledge, MIN_SOC_MAS, PlannedAction, TO_CHARGER_S, VehicleSnapshot, outage_at

class Controller:
    name = "BASE"
    planner_time_limit = 0.9
    last_plan = ()
    def decide(self, now: int, vehicles: tuple[VehicleSnapshot, ...], charger_busy_until: int, knowledge: Knowledge) -> list[PlannedAction]:
        raise NotImplementedError

class StupidController(Controller):
    name = "STUPID"

    def __init__(self):
        self.reset()

    def reset(self):
        self._charger_intent = set()
        self._charger_trip_started = set()
        self._charger_reserved_by = None

    @staticmethod
    def _charge_duration(vehicle, soc):
        remaining = tuple(max(0, duration) for duration in vehicle.tasks[vehicle.task_idx:] if duration > 0)
        if not remaining:
            return 0
        target = min(
            BATTERY_MAS,
            FROM_CHARGER_S * DRIVE_MA + sum(remaining) * DRIVE_MA + CHARGE_TARGET_MAS,
        )
        if soc >= target:
            return 0
        return min(
            (target - soc + CHARGE_MA - 1) // CHARGE_MA,
            max(0, (BATTERY_MAS - soc) // CHARGE_MA),
        )

    def _reservation_release(self, now, vehicles, charger_busy_until, knowledge):
        owner = next(
            (vehicle for vehicle in vehicles if vehicle.id == self._charger_reserved_by),
            None,
        )
        if owner is None or owner.task_idx >= len(owner.tasks):
            self._charger_reserved_by = None
            return now

        if owner.location == "charger" and charger_busy_until > now:
            return charger_busy_until

        ready_at = max(now, owner.busy_until)
        ready_location = owner.ready_location if owner.busy_until > now and owner.ready_location else owner.location
        ready_soc = owner.ready_soc if owner.busy_until > now and owner.ready_soc is not None else owner.soc
        if ready_location != "charger":
            ready_at += TO_CHARGER_S
        ready_at = max(ready_at, charger_busy_until)
        if owner.id not in self._charger_trip_started:
            for outage in sorted(knowledge.observed_outages, key=lambda item: item.start):
                if outage.start <= ready_at < outage.end:
                    ready_at = outage.end
        return ready_at + self._charge_duration(owner, ready_soc)

    def on_action_started(self, action):
        if action.kind == ActionType.DRIVE_TO_CHARGER:
            self._charger_trip_started.add(action.vehicle)
            self._charger_reserved_by = action.vehicle
        elif action.kind == ActionType.DRIVE_FROM_CHARGER:
            self._charger_trip_started.discard(action.vehicle)
            self._charger_intent.discard(action.vehicle)
            if self._charger_reserved_by == action.vehicle:
                self._charger_reserved_by = None

    def decide(self, now, vehicles, charger_busy_until, knowledge):
        actions = []
        for v in vehicles:
            if v.busy_until > now or v.task_idx >= len(v.tasks):
                continue
            if v.location == "charger":
                self._charger_intent.discard(v.id)
                if self._charger_reserved_by is None:
                    self._charger_reserved_by = v.id
            remaining = tuple(max(0, d) for d in v.tasks[v.task_idx:] if d > 0)
            if not remaining:
                continue
            total_drive = sum(remaining) * DRIVE_MA
            next_drive = remaining[0] * DRIVE_MA
            reach_charger = TO_CHARGER_S * DRIVE_MA
            leave_charger = FROM_CHARGER_S * DRIVE_MA
            if v.location == "mission":
                if v.id in self._charger_intent or (
                    v.soc < total_drive + MIN_SOC_MAS
                    and v.soc < next_drive + reach_charger + MIN_SOC_MAS
                ):
                    if v.id not in self._charger_intent and v.soc < reach_charger + MIN_SOC_MAS:
                        continue
                    self._charger_intent.add(v.id)
                    if self._charger_reserved_by is None:
                        self._charger_reserved_by = v.id
                    if self._charger_reserved_by == v.id:
                        arrival = max(now + TO_CHARGER_S, charger_busy_until)
                    else:
                        release_at = self._reservation_release(now, vehicles, charger_busy_until, knowledge)
                        arrival = max(now + TO_CHARGER_S, release_at)
                    for outage in sorted(knowledge.observed_outages, key=lambda item: item.start):
                        if outage.start <= arrival < outage.end:
                            arrival = outage.end
                    departure = arrival - TO_CHARGER_S
                    if departure > now:
                        actions.append(PlannedAction(v.id, ActionType.WAIT, now, departure, v.task_idx))
                    else:
                        self._charger_reserved_by = v.id
                        actions.append(PlannedAction(v.id, ActionType.DRIVE_TO_CHARGER, now, now + TO_CHARGER_S, v.task_idx))
                elif v.soc >= total_drive + MIN_SOC_MAS or v.soc >= next_drive + reach_charger + MIN_SOC_MAS:
                    actions.append(PlannedAction(v.id, ActionType.DELIVERY, now, now + remaining[0], v.task_idx))
                continue
            if charger_busy_until > now:
                wake = max(now, charger_busy_until)
                if wake > now:
                    actions.append(PlannedAction(v.id, ActionType.WAIT, now, wake, v.task_idx))
                continue
            # At the charger: charge enough for all remaining work plus the mandatory
            # drive away from the charger when the remaining work fits in one battery;
            # otherwise charge to full and come back later if needed.
            target = min(BATTERY_MAS, leave_charger + total_drive + CHARGE_TARGET_MAS)
            if v.soc < target:
                outage = outage_at(now, knowledge.observed_outages)
                if outage is not None and v.id not in self._charger_trip_started:
                    actions.append(PlannedAction(v.id, ActionType.WAIT, now, outage.end, v.task_idx))
                    continue
                charge_s = (target - v.soc + CHARGE_MA - 1) // CHARGE_MA
                max_s = max(0, (BATTERY_MAS - v.soc) // CHARGE_MA)
                if max_s > 0:
                    actions.append(PlannedAction(v.id, ActionType.CHARGE, now, now + min(charge_s, max_s), v.task_idx))
                    continue
            if v.soc >= leave_charger + next_drive + MIN_SOC_MAS:
                actions.append(PlannedAction(v.id, ActionType.DRIVE_FROM_CHARGER, now, now + FROM_CHARGER_S, v.task_idx))
        return actions

class OptimizerController(Controller):
    name = "OPTIMIZER"
    def decide(self, now, vehicles, charger_busy_until, knowledge):
        from planner import PlannerInput, first_actions, plan, planner_inputs
        p = planner_inputs(now, vehicles, charger_busy_until, knowledge.observed_outages)
        self.last_plan = tuple(plan(PlannerInput(p.now, p.vehicles, p.charger_busy_until, p.known_outages, self.planner_time_limit)))
        return first_actions(list(self.last_plan), now)

class OracleController(OptimizerController):
    name = "ORACLE"
    def decide(self, now, vehicles, charger_busy_until, knowledge):
        from planner import PlannerInput, first_actions, plan, planner_inputs
        p = planner_inputs(now, vehicles, charger_busy_until, knowledge.all_outages)
        self.last_plan = tuple(plan(PlannerInput(p.now, p.vehicles, p.charger_busy_until, p.known_outages, self.planner_time_limit)))
        return first_actions(list(self.last_plan), now)

class StaticOptimalController(Controller):
    name = "STATIC_OPTIMAL"

    def __init__(self, initial_socs=DEFAULT_INITIAL_SOCS, tasks=DEFAULT_TASKS):
        from planner import PlannerInput, VehicleInput, plan

        nominal_vehicles = tuple(
            VehicleInput(
                id=index + 1,
                now=0,
                soc=int(round(initial_socs[index] * BATTERY_MAS / 100)),
                task_idx=0,
                task_durations=tuple(tasks[index]),
                location="mission",
                ready_at=0,
            )
            for index in range(len(tasks))
        )
        self.schedule = tuple(plan(PlannerInput(0, nominal_vehicles, 0, (), 30.0, 1)))
        self.last_plan = self.schedule
        if not self.schedule:
            raise RuntimeError("Static nominal CP-SAT plan could not be computed")
        self._nominal_slots = tuple(
            (action.vehicle, action.task_idx, action.start, action.end)
            for action in self.schedule
            if action.kind == ActionType.CHARGE
        )
        self._slot_by_action = {
            (vehicle_id, task_idx): slot_id
            for slot_id, (vehicle_id, task_idx, _, _) in enumerate(self._nominal_slots)
        }
        self._by_vehicle = {
            vehicle_id: tuple(
                sorted(
                    (a for a in self.schedule if a.vehicle == vehicle_id and a.kind != ActionType.WAIT),
                    key=lambda a: (a.start, a.end, a.kind.value),
                )
            )
            for vehicle_id in range(1, len(nominal_vehicles) + 1)
        }
        self.reset()

    def reset(self):
        self._next_action = {vehicle_id: 0 for vehicle_id in self._by_vehicle}
        self._advance_on_complete = {vehicle_id: False for vehicle_id in self._by_vehicle}
        self._slots = [
            {"vehicle": vehicle_id, "task_idx": task_idx, "start": start, "end": end, "started": False, "committed": False, "completed": False}
            for vehicle_id, task_idx, start, end in self._nominal_slots
        ]
        self._handled_outages = set()

    def _shift_unstarted_slots(self, slot_index, shift, include_committed_slot=False):
        if shift == 0:
            return
        movable = []
        reserved = []
        for index, slot in enumerate(self._slots):
            can_move = (
                index >= slot_index
                and not slot["started"]
                and not slot["completed"]
                and (not slot["committed"] or (include_committed_slot and index == slot_index))
            )
            if can_move:
                slot["start"] += shift
                slot["end"] += shift
                movable.append(slot)
            else:
                reserved.append((slot["start"], slot["end"]))
        for slot in movable:
            duration = slot["end"] - slot["start"]
            start = slot["start"]
            while True:
                conflicts = [
                    end
                    for reserved_start, end in reserved
                    if start < end and start + duration > reserved_start
                ]
                if not conflicts:
                    break
                start = max(conflicts)
            slot["start"] = start
            slot["end"] = start + duration
            reserved.append((slot["start"], slot["end"]))

    @property
    def current_schedule(self):
        offsets = {}
        actions = []
        for action in self.schedule:
            if action.kind == ActionType.WAIT:
                continue
            slot_id = self._slot_by_action.get((action.vehicle, action.task_idx))
            if action.kind == ActionType.DRIVE_TO_CHARGER and slot_id is not None:
                slot = self._slots[slot_id]
                start = slot["start"] - TO_CHARGER_S
                offsets[action.vehicle] = start - action.start
                actions.append(replace(action, start=start, end=start + action.end - action.start))
            elif action.kind == ActionType.CHARGE and slot_id is not None:
                slot = self._slots[slot_id]
                offsets[action.vehicle] = slot["start"] - action.start
                actions.append(replace(action, start=slot["start"], end=slot["end"]))
            else:
                offset = offsets.get(action.vehicle, 0)
                actions.append(replace(action, start=action.start + offset, end=action.end + offset))
        return tuple(sorted(actions, key=lambda action: (action.start, action.vehicle, action.kind.value)))

    def _shift_slots_for_outages(self, observed_outages, now):
        for outage in sorted(observed_outages, key=lambda o: (o.start, o.end)):
            if outage.start > now or outage.end <= now:
                continue
            key = (outage.start, outage.end)
            if key in self._handled_outages:
                continue
            self._handled_outages.add(key)
            slot_index = next(
                (
                    index
                    for index, slot in enumerate(self._slots)
                    if not slot["started"]
                    and not slot["committed"]
                    and not slot["completed"]
                    and slot["start"] < outage.end
                ),
                None,
            )
            if slot_index is None:
                continue
            slot = self._slots[slot_index]
            self._shift_unstarted_slots(slot_index, outage.end - slot["start"])

    def on_action_started(self, action):
        self._advance_on_complete[action.vehicle] = True
        if action.kind in (ActionType.DRIVE_TO_CHARGER, ActionType.CHARGE):
            slot_id = self._slot_by_action.get((action.vehicle, action.task_idx))
            if slot_id is not None:
                self._slots[slot_id]["committed"] = True
                if action.kind == ActionType.CHARGE:
                    self._slots[slot_id]["started"] = True

    def on_action_completed(self, action):
        vehicle_id = action.vehicle
        if not self._advance_on_complete.get(vehicle_id, False):
            return
        actions = self._by_vehicle[vehicle_id]
        index = self._next_action[vehicle_id]
        if index < len(actions):
            expected = actions[index]
            if expected.kind == action.kind and expected.task_idx == action.task_idx:
                if action.kind == ActionType.CHARGE:
                    slot_id = self._slot_by_action.get((vehicle_id, action.task_idx))
                    if slot_id is not None:
                        self._slots[slot_id]["started"] = False
                        self._slots[slot_id]["completed"] = True
                self._next_action[vehicle_id] += 1
        self._advance_on_complete[vehicle_id] = False

    def decide(self, now, vehicles, charger_busy_until, knowledge):
        self._shift_slots_for_outages(knowledge.observed_outages, now)
        actions = []
        for vehicle in vehicles:
            if vehicle.busy_until > now or vehicle.task_idx >= len(vehicle.tasks):
                continue
            vehicle_id = vehicle.id
            schedule = self._by_vehicle.get(vehicle_id, ())
            index = self._next_action.get(vehicle_id, 0)
            self._next_action[vehicle_id] = index
            if index >= len(schedule):
                continue

            planned = schedule[index]
            self._advance_on_complete[vehicle_id] = False
            if planned.kind == ActionType.DRIVE_TO_CHARGER:
                slot_index = self._slot_by_action.get((vehicle_id, planned.task_idx))
                if slot_index is not None:
                    slot_start = self._slots[slot_index]["start"]
                    departure = slot_start - TO_CHARGER_S
                    if now < departure:
                        actions.append(PlannedAction(vehicle_id, ActionType.WAIT, now, departure, planned.task_idx))
                        continue

            if planned.kind == ActionType.CHARGE:
                slot_id = self._slot_by_action[(vehicle_id, planned.task_idx)]
                slot = self._slots[slot_id]
                if not slot["started"] and now > slot["start"]:
                    self._shift_unstarted_slots(slot_id, now - slot["start"], include_committed_slot=True)
                if now < slot["start"]:
                    self._shift_unstarted_slots(slot_id, now - slot["start"], include_committed_slot=True)
                outage = outage_at(now, knowledge.observed_outages)
                outage_blocks_start = (
                    outage is not None
                    and not slot["committed"]
                    and outage.start <= slot["start"] < outage.end
                )
                unavailable_until = max(
                    charger_busy_until,
                    outage.end if outage_blocks_start else now,
                )
                if unavailable_until > now:
                    self._shift_unstarted_slots(slot_id, unavailable_until - slot["start"], include_committed_slot=True)
                    actions.append(PlannedAction(vehicle_id, ActionType.WAIT, now, unavailable_until, planned.task_idx))
                    continue
                duration = slot["end"] - now
                if duration <= 0:
                    slot["completed"] = True
                    self._next_action[vehicle_id] += 1
                    index += 1
                    if index >= len(schedule):
                        continue
                    planned = schedule[index]
                else:
                    actions.append(PlannedAction(vehicle_id, ActionType.CHARGE, now, slot["end"], planned.task_idx))
                    self._advance_on_complete[vehicle_id] = True
                    continue

            duration = planned.end - planned.start
            actions.append(PlannedAction(vehicle_id, planned.kind, now, now + duration, planned.task_idx))
            self._advance_on_complete[vehicle_id] = True
        return actions
