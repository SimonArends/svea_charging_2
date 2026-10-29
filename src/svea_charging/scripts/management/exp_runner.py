#!/usr/bin/env python3
from __future__ import annotations

import math
import time
import csv
from dataclasses import dataclass, field

import rclpy
import svea_core.rosonic as rx
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from controllers import (
    OracleController,
    OptimizerController,
    StaticOptimalController,
    StupidController,
)
from scenario import (
    BATTERY_MAS,
    DEFAULT_ENERGY_RATES,
    DEFAULT_INITIAL_SOCS,
    DEFAULT_TASKS,
    DRIVE_MA,
    EnergyRates,
    FROM_CHARGER_S,
    WAIT_MA,
    ActionType,
    Knowledge,
    Outage,
    PlannedAction,
    VehicleSnapshot,
)

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
battery_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class VehicleRuntime:
    vehicle_id: int
    tasks: tuple[int, ...]
    soc: float
    completed_tasks: list[CompletedTask] = field(default_factory=list)
    current: float | None = None
    voltage: float | None = None
    battery_updated_at: float | None = None
    soc_updated_at: float | None = None
    status: str | None = None
    status_generation: int = 0
    task_idx: int = 0
    location: str = "mission"
    command: str | None = None
    action: PlannedAction | None = None
    action_started_at: float | None = None
    wait_until: float | None = None
    charge_target: float | None = None
    charge_deadline: float | None = None
    charge_departure_end: int | None = None
    charge_done_sent: bool = False
    resume_action: PlannedAction | None = None
    charger_trip_started_at: int | None = None
    timeline_index: int | None = None
    charge_started_at: float | None = None
    transport_armed: bool = True
    action_soc_start: float | None = None
    transport_length: str | None = None
    transport_length_retry_until: float | None = None
    transport_ready_at: float | None = None


@dataclass(frozen=True)
class CompletedTask:
    task_idx: int
    duration: int
    start: int
    end: int
    soc_start: float
    soc_end: float


@dataclass
class TimelineAction:
    vehicle: int
    kind: ActionType
    task_idx: int | None
    start: int
    planned_end: int
    soc_start_pct: float
    end: int | None = None
    soc_end_pct: float | None = None


class exp_runner(rx.Node):
    update_rate = rx.Parameter(1.0)
    controller_name = rx.Parameter("STUPID")
    planner_time_limit = rx.Parameter(0.9)
    soc_penalty_weight = rx.Parameter(50000)
    total_time_weight = rx.Parameter(100)
    drive_ma = rx.Parameter(DRIVE_MA)
    charge_ma = rx.Parameter(DEFAULT_ENERGY_RATES.charge_ma)
    wait_ma = rx.Parameter(WAIT_MA)
    initial_soc_pct_a = rx.Parameter(float(DEFAULT_INITIAL_SOCS[0]))
    initial_soc_pct_b = rx.Parameter(float(DEFAULT_INITIAL_SOCS[1]))
    tasks_a = rx.Parameter(",".join(str(value) for value in DEFAULT_TASKS[0]))
    tasks_b = rx.Parameter(",".join(str(value) for value in DEFAULT_TASKS[1]))
    # Comma-separated start:end pairs in seconds from experiment start.
    outage_slots = rx.Parameter("")
    battery_stale_after_s = rx.Parameter(2.5)
    charge_timeout_factor = rx.Parameter(3.0)
    transport_reset_wait_s = rx.Parameter(2.0)
    transport_length_retry_s = rx.Parameter(2.0)

    battery_charging_topic_a = rx.Parameter("/svea_a/mavros/battery")
    battery_charging_topic_b = rx.Parameter("/svea_b/mavros/battery")
    vehicle_status_topic_a = rx.Parameter("/svea_a/mission/vehicle_status")
    vehicle_status_topic_b = rx.Parameter("/svea_b/mission/vehicle_status")
    experiment_info_topic = rx.Parameter("/experiment/info")

    charge_complete_pub_a = rx.Publisher(Bool, "/svea_a/mission/charge_done", qos_pubber)
    charge_complete_pub_b = rx.Publisher(Bool, "/svea_b/mission/charge_done", qos_pubber)
    experiment_info_pub = rx.Publisher(String, experiment_info_topic, qos_pubber)
    action_pub_a = rx.Publisher(String, "/svea_a/mission/action")
    action_pub_b = rx.Publisher(String, "/svea_b/mission/action")
    tr_length_pub_a = rx.Publisher(String, "/svea_a/mission/transport_length")
    tr_length_pub_b = rx.Publisher(String, "/svea_b/mission/transport_length")

    @rx.Subscriber(BatteryState, battery_charging_topic_a, battery_qos)
    def _battery_charging_cb_a(self, msg: BatteryState):
        self._battery_update(1, msg)

    @rx.Subscriber(BatteryState, battery_charging_topic_b, battery_qos)
    def _battery_charging_cb_b(self, msg: BatteryState):
        self._battery_update(2, msg)

    @rx.Subscriber(String, vehicle_status_topic_a, qos_pubber)
    def vehicle_status_a_cb(self, msg: String):
        self._status_update(1, msg.data)

    @rx.Subscriber(String, vehicle_status_topic_b, qos_pubber)
    def vehicle_status_b_cb(self, msg: String):
        self._status_update(2, msg.data)

    def on_startup(self):
        self._validate_parameters()
        self._started_at = None
        self._last_loop_at = None
        self._previous_outage_state = False
        self._replan_requested = True
        self._latest_plan_actions = ()
        self._run_csv_file = None
        self._run_csv_writer = None
        self._run_csv_closed = False
        self._last_run_sample_second = None
        self._timeline: list[TimelineAction] = []
        self._charging_visits = 0
        self._failure_reason: str | None = None
        self._completed_at: int | None = None
        self._runtimes = {
            1: VehicleRuntime(1, self._tasks_a, self._initial_soc_a),
            2: VehicleRuntime(2, self._tasks_b, self._initial_soc_b),
        }
        self._outages = self._parse_outages(str(self.outage_slots))
        self._controller = self._make_controller()
        self._controller.planner_time_limit = float(self.planner_time_limit)
        self._controller.soc_penalty_weight = int(self.soc_penalty_weight)
        self._controller.total_time_weight = int(self.total_time_weight)
        self._publishers = {
            1: (self.action_pub_a, self.tr_length_pub_a, self.charge_complete_pub_a),
            2: (self.action_pub_b, self.tr_length_pub_b, self.charge_complete_pub_b),
        }
        period = 1.0 / float(self.update_rate)
        self.create_timer(period, self.loop)
        self.create_timer(1.0, self._record_run_sample)
        self.get_logger().info(
            f"Experiment runner: {self._controller.name}; "
            f"initial SoC=({self._initial_soc_a:.1f}%, {self._initial_soc_b:.1f}%); "
            f"energy rates=drive {self._energy_rates.drive_ma} mA, "
            f"charge {self._energy_rates.charge_ma} mA, wait {self._energy_rates.wait_ma} mA; "
            f"outages={[(o.start, o.end) for o in self._outages]}"
        )


    def _validate_parameters(self):
        if float(self.update_rate) <= 0:
            raise ValueError("update_rate must be positive")
        if float(self.battery_stale_after_s) <= 0:
            raise ValueError("battery_stale_after_s must be positive")
        if float(self.charge_timeout_factor) < 1:
            raise ValueError("charge_timeout_factor must be at least 1")
        if float(self.transport_reset_wait_s) < 0:
            raise ValueError("transport_reset_wait_s must not be negative")
        if float(self.transport_length_retry_s) <= 0:
            raise ValueError("transport_length_retry_s must be positive")
        self._initial_soc_a = float(self.initial_soc_pct_a)
        self._initial_soc_b = float(self.initial_soc_pct_b)
        if any(not math.isfinite(value) or not 0 <= value <= 100 for value in (self._initial_soc_a, self._initial_soc_b)):
            raise ValueError("initial SoC values must be between 0 and 100 percent")
        self._tasks_a = self._parse_tasks(str(self.tasks_a), "tasks_a")
        self._tasks_b = self._parse_tasks(str(self.tasks_b), "tasks_b")
        self._energy_rates = EnergyRates(
            drive_ma=int(self.drive_ma),
            charge_ma=int(self.charge_ma),
            wait_ma=int(self.wait_ma),
        )
        if float(self.planner_time_limit) <= 0:
            raise ValueError("planner_time_limit must be positive")
        if int(self.soc_penalty_weight) < 0:
            raise ValueError("soc_penalty_weight must not be negative")
        if int(self.total_time_weight) < 0:
            raise ValueError("total_time_weight must not be negative")

    @staticmethod
    def _parse_tasks(value: str, name: str) -> tuple[int, ...]:
        try:
            tasks = tuple(int(item.strip()) for item in value.split(",") if item.strip())
        except ValueError as error:
            raise ValueError(f"{name} must be comma-separated task durations") from error
        if not tasks or any(task not in (60, 110, 160) for task in tasks):
            raise ValueError(f"{name} durations must be 60, 110, or 160 seconds")
        return tasks

    @staticmethod
    def _parse_outages(value: str) -> tuple[Outage, ...]:
        outages = []
        for item in value.split(","):
            if not item.strip():
                continue
            try:
                start, end = (int(part.strip()) for part in item.split(":"))
            except (TypeError, ValueError) as error:
                raise ValueError("outage_slots must look like '300:420,900:1020'") from error
            if start < 0 or end <= start:
                raise ValueError(f"invalid outage interval: {item}")
            outages.append(Outage(start, end))
        outages.sort(key=lambda outage: (outage.start, outage.end))
        for previous, current in zip(outages, outages[1:]):
            if current.start < previous.end:
                raise ValueError("outage_slots must not overlap")
        return tuple(outages)

    def _make_controller(self):
        classes = {
            "STUPID": StupidController,
            "OPTIMIZER": OptimizerController,
            "ORACLE": OracleController,
            "STATIC_OPTIMAL": StaticOptimalController,
        }
        name = str(self.controller_name).upper()
        if name not in classes:
            raise ValueError(f"controller_name must be one of {tuple(classes)}")
        if name == "STATIC_OPTIMAL":
            return StaticOptimalController(
                initial_socs=(self._initial_soc_a, self._initial_soc_b),
                tasks=(self._tasks_a, self._tasks_b),
                    soc_penalty_weight=int(self.soc_penalty_weight),
                total_time_weight=int(self.total_time_weight),
                energy_rates=self._energy_rates,
            )
        return classes[name](energy_rates=self._energy_rates)

    def _battery_update(self, vehicle_id: int, msg: BatteryState):
        runtime = getattr(self, "_runtimes", {}).get(vehicle_id)
        if runtime is None:
            return
        runtime.current = float(msg.current)
        runtime.voltage = float(msg.voltage)
        runtime.battery_updated_at = time.monotonic()

    def _start_experiment(self, now: float):
        if self._started_at is not None:
            return
        self._started_at = now
        self._last_loop_at = now
        self._previous_outage_state = self._outage_at(0) is not None
        for runtime in self._runtimes.values():
            runtime.soc_updated_at = now
        self.get_logger().info("Experiment started with first vehicle action")

    def _status_update(self, vehicle_id: int, status: str):
        runtime = getattr(self, "_runtimes", {}).get(vehicle_id)
        if runtime is None:
            return
        if status != runtime.status:
            runtime.status = status
            runtime.status_generation += 1
            if runtime.command == "transport" and status != "transport_done":
                runtime.transport_armed = True
            self._replan_requested = True

    def _publish_action(self, vehicle_id: int, command: str):
        action_pub, _, _ = self._publishers[vehicle_id]
        action_pub.publish(String(data=command))

    def _publish_charge_complete(self, vehicle_id: int, complete: bool):
        _, _, complete_pub = self._publishers[vehicle_id]
        complete_pub.publish(Bool(data=complete))

    def _publish_runtime_commands(self, now: float):
        for runtime in self._runtimes.values():
            if (
                runtime.command == "transport"
                and runtime.transport_length is not None
                and runtime.transport_length_retry_until is not None
                and now < runtime.transport_length_retry_until
            ):
                _, length_pub, _ = self._publishers[runtime.vehicle_id]
                length_pub.publish(String(data=runtime.transport_length))
            self._publish_action(runtime.vehicle_id, runtime.command or "wait")
            self._publish_charge_complete(runtime.vehicle_id, runtime.charge_done_sent)

    def _update_soc(self, now: float):
        if self._started_at is None:
            return
        stale_after = float(self.battery_stale_after_s)
        for runtime in self._runtimes.values():
            if runtime.task_idx >= len(runtime.tasks):
                continue
            previous = runtime.soc_updated_at or now
            valid_until = min(now, (runtime.battery_updated_at or previous) + stale_after)
            if runtime.current is not None and valid_until > previous:
                # BatteryState.current is positive while charging and negative while discharging.
                delta_pct = runtime.current * 1000.0 * (valid_until - previous) * 100.0 / BATTERY_MAS
                runtime.soc = min(100.0, max(0.0, runtime.soc + delta_pct))
            runtime.soc_updated_at = now

    def _stop_for_empty_battery(self, now: float) -> bool:
        depleted = [
            runtime
            for runtime in self._runtimes.values()
            if runtime.task_idx < len(runtime.tasks) and runtime.soc <= 0.0
        ]
        if not depleted:
            return False
        vehicle_ids = ", ".join(f"V{runtime.vehicle_id}" for runtime in depleted)
        self.get_logger().error(
            f"Experiment aborted at t={self._elapsed(now)}s: virtual SoC reached 0% for {vehicle_ids}"
        )
        self._failure_reason = f"virtual SoC reached 0% for {vehicle_ids}"
        for runtime in self._runtimes.values():
            self._publish_action(runtime.vehicle_id, "wait")
        rclpy.shutdown()
        return True

    def _elapsed(self, now: float) -> int:
        if self._started_at is None:
            return 0
        return max(0, int(now - self._started_at))

    @staticmethod
    def _format_time(seconds: int) -> str:
        hours, remainder = divmod(max(0, int(seconds)), 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

    def _current_action(self, runtime: VehicleRuntime) -> str:
        if runtime.task_idx >= len(runtime.tasks):
            return "COMPLETE"
        if runtime.command == "transport":
            task_idx = (
                runtime.action.task_idx
                if runtime.action is not None and runtime.action.task_idx is not None
                else runtime.task_idx
            )
            return f"Delivering task {task_idx + 1}/{len(runtime.tasks)}"
        if runtime.command == "wait":
            if runtime.wait_until is not None:
                return f"Waiting until {self._format_time(self._elapsed(runtime.wait_until))}"
            return "Waiting"
        if runtime.command == "charge_cycle":
            if runtime.wait_until is not None:
                return f"Waiting until {self._format_time(self._elapsed(runtime.wait_until))}"
            if runtime.charge_target is not None:
                return f"Charging (target {runtime.charge_target:.1f}%)"
            if runtime.charge_done_sent:
                return "Charge complete; leaving charger"
            if runtime.status == "docked":
                return "At charger"
            if runtime.action is not None and runtime.action.kind == ActionType.DRIVE_TO_CHARGER:
                return "Driving to charger"
            return f"Charge cycle ({runtime.status or 'awaiting status'})"
        return f"Idle ({runtime.status or 'awaiting status'})"

    def _experiment_info(self, now: float) -> str:
        elapsed = self._elapsed(now)
        all_complete = all(
            runtime.task_idx >= len(runtime.tasks)
            for runtime in self._runtimes.values()
        )
        if all_complete and self._completed_at is None:
            self._completed_at = elapsed
        if self._completed_at is not None:
            elapsed = self._completed_at
        experiment_state = (
            "COMPLETE"
            if all_complete
            else "READY" if self._started_at is None else "RUNNING"
        )
        lines = [
            f"EXPERIMENT {experiment_state}",
            f"Elapsed: {self._format_time(elapsed)} ({elapsed}s) | Controller: {self._controller.name}",
        ]

        active_outage = self._outage_at(elapsed)
        if active_outage is None:
            lines.append("Charger: AVAILABLE")
        else:
            lines.append(
                f"Charger: OUTAGE until {self._format_time(active_outage.end)} "
                f"({active_outage.end}s)"
            )

        if self._outages:
            outage_summary = []
            for outage in self._outages:
                if elapsed < outage.start:
                    state = "upcoming"
                elif elapsed < outage.end:
                    state = "active"
                else:
                    state = "ended"
                outage_summary.append(
                    f"{self._format_time(outage.start)}-{self._format_time(outage.end)} {state}"
                )
            lines.append("Outages: " + "; ".join(outage_summary))
        else:
            lines.append("Outages: none configured")

        lines.append("Vehicles:")
        for runtime in self._runtimes.values():
            battery_details = []
            if runtime.current is not None:
                battery_details.append(f"I {runtime.current:+.2f}A")
            if runtime.voltage is not None:
                battery_details.append(f"{runtime.voltage:.2f}V")
            battery_suffix = f" | {' '.join(battery_details)}" if battery_details else ""
            lines.append(
                f"  V{runtime.vehicle_id}: SoC {runtime.soc:.1f}% | "
                f"{self._current_action(runtime)} | status {runtime.status or 'unknown'}"
                f"{battery_suffix}"
            )

        durations = {kind: 0 for kind in ActionType}
        for entry in self._timeline:
            end = entry.end if entry.end is not None else elapsed
            durations[entry.kind] += max(0, end - entry.start)
        transport_fraction = (
            durations[ActionType.DELIVERY] / (len(self._runtimes) * elapsed)
            if elapsed else 0.0
        )
        charger_utilization = durations[ActionType.CHARGE] / elapsed if elapsed else 0.0
        lines.extend([
            "Results:",
            f"  Completed: {all_complete}",
            f"  Makespan: {elapsed} s",
            f"  Total time spent waiting: {durations[ActionType.WAIT]} s",
            f"  Transport time fraction: {100 * transport_fraction:.1f}%",
            f"  Charger utilization: {100 * charger_utilization:.1f}%",
            f"  Charging visits: {self._charging_visits}",
            f"  Failure reason: {self._failure_reason or 'None'}",
        ])
        timeline_lines = ["TIMELINE:"]
        if not self._timeline:
            timeline_lines.append("  NO ACTIONS STARTED")
        else:
            current_soc = {
                runtime.vehicle_id: runtime.soc
                for runtime in self._runtimes.values()
            }
            for entry in sorted(self._timeline, key=lambda item: (item.start, item.vehicle, item.kind.value)):
                end = entry.end if entry.end is not None else entry.planned_end
                duration = max(0, end - entry.start)
                action_name = entry.kind.value.replace("_", " ")
                task = f" TASK {entry.task_idx + 1}" if entry.task_idx is not None else ""
                soc_end = entry.soc_end_pct
                if soc_end is None and entry.end is None:
                    soc_end = current_soc.get(entry.vehicle)
                soc_detail = (
                    f" | SOC {entry.soc_start_pct:.1f}%->{soc_end:.1f}%"
                    if soc_end is not None else ""
                )
                progress = " (IN PROGRESS; END FORECAST)" if entry.end is None else ""
                timeline_lines.append(
                    f"  V{entry.vehicle} {action_name}{task} "
                    f"{self._format_time(entry.start)}-{self._format_time(end)} "
                    f"({duration}S){soc_detail}{progress}"
                )

        if self._controller.name == "STATIC_OPTIMAL":
            plan_actions = self._controller.current_schedule
            plan_label = "Static optimal schedule"
        elif self._controller.name in {"OPTIMIZER", "ORACLE"}:
            plan_actions = self._controller.last_plan
            plan_label = f"{self._controller.name} full remaining plan"
            if self._controller.last_plan_stale:
                plan_label += " (last feasible; latest replan failed)"
        else:
            plan_actions = self._latest_plan_actions
            plan_label = "Latest immediate plan"
        active_actions = []
        for runtime in self._runtimes.values():
            if runtime.command == "charge_cycle" and runtime.charge_done_sent:
                departure_end = runtime.charge_departure_end or elapsed + 45
                active_actions.append(
                    PlannedAction(
                        runtime.vehicle_id,
                        ActionType.DRIVE_FROM_CHARGER,
                        departure_end - 45,
                        departure_end,
                        runtime.task_idx,
                    )
                )
                continue
            action = runtime.action
            if action is None:
                continue
            active = (
                (runtime.command == "transport" and action.kind == ActionType.DELIVERY)
                or (runtime.command == "wait" and action.kind == ActionType.WAIT)
                or (
                    runtime.command == "charge_cycle"
                    and (
                        (action.kind == ActionType.DRIVE_TO_CHARGER and runtime.status != "docked")
                        or (
                            action.kind == ActionType.CHARGE
                            and runtime.charge_target is not None
                            and not runtime.charge_done_sent
                        )
                    )
                )
            )
            if active and action not in plan_actions:
                active_actions.append(action)
        active_keys = {
            (action.vehicle, action.kind, action.task_idx)
            for action in active_actions
        }
        plan_actions = tuple(
            sorted(
                (*active_actions, *(action for action in plan_actions if (action.vehicle, action.kind, action.task_idx) not in active_keys)),
                key=lambda action: (action.start, action.vehicle, action.kind.value),
            )
        )
        upcoming = [
            action
            for action in plan_actions
            if action.end > elapsed or action in active_actions
        ]
        lines.append(f"{plan_label}:")
        if upcoming:
            for action in upcoming:
                action_name = action.kind.value.replace("_", " ").lower()
                task_label = (
                    f" task {action.task_idx + 1}"
                    if action.task_idx is not None and action.kind == ActionType.DELIVERY
                    else ""
                )
                duration = action.end - action.start
                soc_detail = ""
                if action.soc_start_pct is not None and action.soc_end_pct is not None:
                    soc_detail = (
                        f" | SoC {action.soc_start_pct:.1f}%"
                        f"->{action.soc_end_pct:.1f}%"
                    )
                progress = " (in progress)" if action.start < elapsed else ""
                lines.append(
                    f"  V{action.vehicle} {action_name}{task_label} "
                    f"{self._format_time(action.start)}-{self._format_time(action.end)} "
                    f"({duration}s){soc_detail}{progress}"
                )
        else:
            lines.append("  No future plan currently available")
        lines.extend(timeline_lines)
        return "\n".join(lines)

    def _publish_experiment_info(self, now: float):
        self.experiment_info_pub.publish(String(data=self._experiment_info(now)))

    def _record_run_sample(self):
        if self._started_at is None or self._run_csv_closed:
            return
        now = time.monotonic()
        self._update_soc(now)
        elapsed = self._elapsed(now)
        if elapsed == self._last_run_sample_second:
            return
        if self._run_csv_writer is None:
            self._run_csv_file = open("run.csv", "w", newline="")
            self._run_csv_writer = csv.DictWriter(
                self._run_csv_file,
                fieldnames=(
                    "elapsed_s",
                    "charger_status",
                    "vehicle_1_action",
                    "vehicle_1_soc_pct",
                    "vehicle_2_action",
                    "vehicle_2_soc_pct",
                ),
            )
            self._run_csv_writer.writeheader()

        charger_status = "OUTAGE" if self._outage_at(elapsed) is not None else "AVAILABLE"
        if charger_status == "AVAILABLE":
            for runtime in self._runtimes.values():
                if runtime.status == "docked" and runtime.command == "charge_cycle":
                    charger_status = (
                        f"CHARGING_V{runtime.vehicle_id}"
                        if runtime.charge_target is not None and not runtime.charge_done_sent
                        else f"OCCUPIED_V{runtime.vehicle_id}"
                    )
                    break

        row = {"elapsed_s": elapsed, "charger_status": charger_status}
        for runtime in self._runtimes.values():
            row[f"vehicle_{runtime.vehicle_id}_action"] = self._current_action(runtime)
            row[f"vehicle_{runtime.vehicle_id}_soc_pct"] = round(runtime.soc, 3)
        self._run_csv_writer.writerow(row)
        self._run_csv_file.flush()
        self._last_run_sample_second = elapsed

        if self._failure_reason is not None or all(
            runtime.task_idx >= len(runtime.tasks)
            for runtime in self._runtimes.values()
        ):
            self._run_csv_file.close()
            self._run_csv_file = None
            self._run_csv_writer = None
            self._run_csv_closed = True

    def _outage_at(self, elapsed: int) -> Outage | None:
        return next((outage for outage in self._outages if outage.start <= elapsed < outage.end), None)

    def _knowledge(self, elapsed: int) -> Knowledge:
        observed = tuple(outage for outage in self._outages if outage.start <= elapsed)
        return Knowledge(observed, self._outages)

    def _charge_busy_until(
        self,
        now: int,
        planning_vehicle: int | None = None,
        flexible_charges: bool = False,
    ) -> int:
        busy_until = now
        for runtime in self._runtimes.values():
            if runtime.vehicle_id == planning_vehicle or runtime.command != "charge_cycle":
                continue
            if runtime.charge_done_sent or runtime.status == "parked_after_charge":
                continue
            if runtime.status == "docked" and runtime.charge_target is None:
                continue
            if flexible_charges and runtime.status == "docked" and runtime.charge_target is not None:
                continue
            if runtime.status == "docked" and runtime.charge_target is not None:
                remaining = max(
                    0,
                    math.ceil(
                        (runtime.charge_target - runtime.soc)
                        * BATTERY_MAS
                        / (100.0 * self._energy_rates.charge_ma)
                    ),
                )
                busy_until = max(busy_until, now + remaining)
            elif runtime.status != "parked_after_charge":
                busy_until = max(busy_until, now + 45)
        return busy_until

    def _snapshots(self, now: int, flexible_charges: bool = False) -> tuple[VehicleSnapshot, ...]:
        snapshots = []
        monotonic_now = time.monotonic()
        for runtime in self._runtimes.values():
            if runtime.task_idx >= len(runtime.tasks):
                continue
            busy_until = now
            ready_soc = int(round(runtime.soc * BATTERY_MAS / 100.0))
            ready_task_idx = runtime.task_idx
            ready_location = runtime.location
            charging = False
            charger_trip_started_at = runtime.charger_trip_started_at

            if runtime.wait_until is not None:
                remaining = max(0.0, runtime.wait_until - monotonic_now)
                busy_until = now + int(math.ceil(remaining))
                ready_soc -= int(round(remaining * self._energy_rates.wait_ma))
            elif runtime.command == "transport" and runtime.action is not None:
                duration = runtime.action.end - runtime.action.start
                finish_at = (runtime.action_started_at or monotonic_now) + duration
                remaining = max(0.0, finish_at - monotonic_now)
                busy_until = now + int(math.ceil(remaining))
                ready_soc -= int(round(remaining * self._energy_rates.drive_ma))
                ready_task_idx += 1
                ready_location = "mission"
            elif runtime.command == "charge_cycle":
                if runtime.charge_done_sent:
                    departure_end = runtime.charge_departure_end or now + 45
                    remaining = max(0, departure_end - now)
                    ready_soc -= remaining * self._energy_rates.drive_ma
                    ready_location = "mission"
                elif runtime.status == "docked" and runtime.charge_target is not None:
                    if flexible_charges:
                        remaining = 0.0
                        ready_soc = int(round(runtime.soc * BATTERY_MAS / 100.0))
                        ready_location = "charger"
                        charging = True
                    else:
                        charge_remaining = max(
                            0.0,
                            (runtime.charge_target - runtime.soc)
                            * BATTERY_MAS
                            / (100.0 * self._energy_rates.charge_ma),
                        )
                        remaining = charge_remaining + 45.0
                        ready_soc = int(round(runtime.charge_target * BATTERY_MAS / 100.0)) - FROM_CHARGER_S * self._energy_rates.drive_ma
                        ready_location = "mission"
                elif runtime.status == "docked":
                    remaining = 0.0
                    ready_location = "charger"
                else:
                    duration = (
                        runtime.action.end - runtime.action.start
                        if runtime.action is not None
                        else 45
                    )
                    finish_at = (runtime.action_started_at or monotonic_now) + duration
                    remaining = max(0.0, finish_at - monotonic_now)
                    ready_soc -= int(round(remaining * self._energy_rates.drive_ma))
                    ready_location = "charger"
                    if runtime.action is not None and runtime.action.kind == ActionType.DRIVE_TO_CHARGER:
                        charger_trip_started_at = self._elapsed(runtime.action_started_at or monotonic_now)
                busy_until = now + int(math.ceil(remaining))

            snapshots.append(
                VehicleSnapshot(
                    runtime.vehicle_id,
                    int(round(runtime.soc * BATTERY_MAS / 100.0)),
                    runtime.tasks,
                    runtime.task_idx,
                    runtime.location,
                    busy_until,
                    max(0, min(BATTERY_MAS, ready_soc)),
                    ready_task_idx,
                    ready_location,
                    charging,
                    charger_trip_started_at,
                )
            )
        return tuple(snapshots)

    def _notify_action_started(self, action: PlannedAction):
        callback = getattr(self._controller, "on_action_started", None)
        if callback is not None:
            callback(action)

    def _notify_action_completed(self, action: PlannedAction, now: int):
        callback = getattr(self._controller, "on_action_completed", None)
        if callback is not None:
            callback(PlannedAction(action.vehicle, action.kind, action.start, now, action.task_idx))

    def _timeline_start(self, runtime: VehicleRuntime, kind: ActionType, task_idx: int | None, now: float, duration: int):
        start = self._elapsed(now)
        entry = TimelineAction(
            runtime.vehicle_id,
            kind,
            task_idx,
            start,
            start + max(0, duration),
            runtime.soc,
        )
        self._timeline.append(entry)
        runtime.timeline_index = len(self._timeline) - 1

    def _timeline_update_forecast(self, runtime: VehicleRuntime, now: float, duration: int):
        if runtime.timeline_index is not None:
            self._timeline[runtime.timeline_index].planned_end = self._elapsed(now) + max(0, duration)

    def _timeline_finish(self, runtime: VehicleRuntime, now: float):
        if runtime.timeline_index is None:
            return
        entry = self._timeline[runtime.timeline_index]
        entry.end = self._elapsed(now)
        entry.soc_end_pct = runtime.soc
        runtime.timeline_index = None

    def _begin_wait(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
        self._timeline_start(runtime, ActionType.WAIT, action.task_idx, now, action.end - action.start)
        runtime.command = "wait"
        runtime.action = action
        runtime.action_started_at = now
        runtime.wait_until = now + max(0, action.end - action.start)
        self._publish_action(runtime.vehicle_id, "wait")
        self._notify_action_started(action)
        if runtime.wait_until <= now:
            self._finish_wait(runtime, now)

    def _finish_wait(self, runtime: VehicleRuntime, now: float):
        action = runtime.action
        self._timeline_finish(runtime, now)
        self._publish_action(runtime.vehicle_id, "wait")
        runtime.command = None
        runtime.action = None
        runtime.action_started_at = None
        runtime.wait_until = None
        if action is not None:
            self._notify_action_completed(action, self._elapsed(now))
        self._replan_requested = True

    def _begin_charger_wait(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
        self._timeline_start(runtime, ActionType.WAIT, action.task_idx, now, action.end - action.start)
        runtime.resume_action = runtime.action
        runtime.action = action
        runtime.action_started_at = now
        runtime.wait_until = now + max(0, action.end - action.start)
        self._notify_action_started(action)
        if runtime.wait_until <= now:
            self._finish_charger_wait(runtime, now)

    def _finish_charger_wait(self, runtime: VehicleRuntime, now: float):
        action = runtime.action
        self._timeline_finish(runtime, now)
        runtime.action = runtime.resume_action
        runtime.resume_action = None
        runtime.action_started_at = None
        runtime.wait_until = None
        if action is not None:
            self._notify_action_completed(action, self._elapsed(now))
        self._replan_requested = True

    def _start_transport(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
        duration = runtime.tasks[runtime.task_idx]
        self._timeline_start(runtime, ActionType.DELIVERY, action.task_idx, now, duration)
        length = {60: "short", 110: "mid", 160: "long"}[duration]
        runtime.transport_length = length
        runtime.transport_length_retry_until = now + float(self.transport_length_retry_s)
        runtime.command = "transport"
        runtime.action = action
        runtime.action_started_at = now
        runtime.action_soc_start = runtime.soc
        runtime.transport_armed = runtime.status != "transport_done"
        _, length_pub, _ = self._publishers[runtime.vehicle_id]
        length_pub.publish(String(data=length))
        self._publish_action(runtime.vehicle_id, "transport")
        self._notify_action_started(action)

    def _start_charge_cycle(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
        self._timeline_start(runtime, ActionType.DRIVE_TO_CHARGER, action.task_idx, now, action.end - action.start)
        runtime.charger_trip_started_at = self._elapsed(now)
        runtime.command = "charge_cycle"
        runtime.action = action
        runtime.action_started_at = now
        runtime.location = "mission"
        runtime.charge_target = None
        runtime.charge_deadline = None
        runtime.charge_departure_end = None
        runtime.charge_done_sent = False
        runtime.resume_action = None
        runtime.charge_started_at = None
        self._publish_charge_complete(runtime.vehicle_id, False)
        self._publish_action(runtime.vehicle_id, "charge_cycle")
        self._notify_action_started(action)

    def _start_charging(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
        duration = max(1, action.end - action.start)
        if runtime.charge_started_at is None:
            runtime.charge_started_at = now
            self._charging_visits += 1
            self._timeline_start(runtime, ActionType.CHARGE, action.task_idx, now, duration)
        else:
            self._timeline_update_forecast(runtime, now, duration)
        runtime.action = action
        charge_gain_pct = self._energy_rates.charge_ma * duration * 100.0 / BATTERY_MAS
        runtime.charge_target = min(100.0, runtime.soc + charge_gain_pct)
        runtime.charge_deadline = now + duration * float(self.charge_timeout_factor) + 60.0
        self._notify_action_started(action)
        self.get_logger().info(
            f"V{runtime.vehicle_id}: charging target {runtime.charge_target:.1f}%"
        )

    def _execute(self, action: PlannedAction, now: float):
        runtime = self._runtimes.get(action.vehicle)
        if runtime is None:
            return
        if action.kind == ActionType.DELIVERY:
            if runtime.transport_ready_at is not None and now < runtime.transport_ready_at:
                elapsed = self._elapsed(now)
                wait = PlannedAction(
                    runtime.vehicle_id,
                    ActionType.WAIT,
                    elapsed,
                    elapsed + max(1, math.ceil(runtime.transport_ready_at - now)),
                    runtime.task_idx,
                )
                self._begin_wait(runtime, wait, now)
                return
            self._start_transport(runtime, action, now)
        elif action.kind == ActionType.DRIVE_TO_CHARGER:
            self._start_charge_cycle(runtime, action, now)
        elif action.kind == ActionType.CHARGE:
            if runtime.command == "charge_cycle" and runtime.status == "docked":
                self._start_charging(runtime, action, now)
        elif action.kind == ActionType.DRIVE_FROM_CHARGER:
            if runtime.command == "charge_cycle" and runtime.status == "docked":
                self._send_charge_complete(runtime)
        elif action.kind == ActionType.WAIT:
            if runtime.command == "charge_cycle" and runtime.status == "docked":
                self._begin_charger_wait(runtime, action, now)
            else:
                self._begin_wait(runtime, action, now)

    def _replan(self, now: float):
        elapsed = self._elapsed(now)
        flexible_charges = self._controller.name in {"OPTIMIZER", "ORACLE"}
        snapshots = self._snapshots(elapsed, flexible_charges=flexible_charges)
        if not snapshots:
            self._latest_plan_actions = ()
            if self._controller.name != "STATIC_OPTIMAL":
                self._controller.last_plan = ()
                self._controller.last_plan_stale = False
            return
        actions = self._controller.decide(
            elapsed,
            snapshots,
            self._charge_busy_until(
                elapsed,
                flexible_charges=flexible_charges,
            ),
            self._knowledge(elapsed),
        )
        self._latest_plan_actions = tuple(actions)
        for action in actions:
            runtime = self._runtimes.get(action.vehicle)
            if runtime is None or runtime.status is None:
                continue
            at_charge_decision = (
                runtime.command == "charge_cycle"
                and runtime.status == "docked"
                and (
                    runtime.charge_target is None
                    or (
                        self._controller.name in {"OPTIMIZER", "ORACLE"}
                        and not runtime.charge_done_sent
                    )
                )
            )
            at_charge_decision = at_charge_decision and runtime.wait_until is None
            if runtime.command is not None and not at_charge_decision:
                continue
            self._execute(action, now)

    def _complete_transport(self, runtime: VehicleRuntime, now: float):
        action = runtime.action
        self._timeline_finish(runtime, now)
        task_idx = action.task_idx if action is not None and action.task_idx is not None else runtime.task_idx
        runtime.completed_tasks.append(
            CompletedTask(
                task_idx,
                runtime.tasks[task_idx],
                self._elapsed(runtime.action_started_at or now),
                self._elapsed(now),
                runtime.action_soc_start if runtime.action_soc_start is not None else runtime.soc,
                runtime.soc,
            )
        )
        self._publish_action(runtime.vehicle_id, "wait")
        runtime.command = None
        runtime.action = None
        runtime.action_started_at = None
        runtime.action_soc_start = None
        runtime.task_idx += 1
        runtime.location = "mission"
        runtime.transport_ready_at = now + float(self.transport_reset_wait_s)
        if action is not None:
            self._notify_action_completed(action, self._elapsed(now))
        self._replan_requested = True
        self.get_logger().info(f"V{runtime.vehicle_id}: delivery {runtime.task_idx} completed")

    def _send_charge_complete(self, runtime: VehicleRuntime):
        if runtime.charge_done_sent:
            return
        now = time.monotonic()
        if runtime.charge_started_at is not None:
            self._timeline_finish(runtime, now)
            runtime.charge_started_at = None
        runtime.charge_done_sent = True
        runtime.charge_departure_end = self._elapsed(now) + 45
        runtime.charger_trip_started_at = None
        self._timeline_start(runtime, ActionType.DRIVE_FROM_CHARGER, runtime.task_idx, now, 45)
        self._publish_charge_complete(runtime.vehicle_id, True)
        if runtime.action is not None and runtime.action.kind == ActionType.CHARGE:
            self._notify_action_completed(runtime.action, self._elapsed(time.monotonic()))
        for waiting_runtime in self._runtimes.values():
            if waiting_runtime.command == "wait" and waiting_runtime.wait_until is not None:
                self._finish_wait(waiting_runtime, now)
            elif waiting_runtime.command == "charge_cycle" and waiting_runtime.wait_until is not None:
                self._finish_charger_wait(waiting_runtime, now)

    def _on_docked(self, runtime: VehicleRuntime, now: float):
        runtime.location = "charger"
        previous = runtime.action
        self._timeline_finish(runtime, now)
        if previous is not None and previous.kind == ActionType.DRIVE_TO_CHARGER:
            self._notify_action_completed(previous, self._elapsed(now))
        runtime.charge_target = None
        runtime.charge_done_sent = False
        self._replan_requested = True

    def _on_parked(self, runtime: VehicleRuntime, now: float):
        self._timeline_finish(runtime, now)
        charge_action = runtime.action
        if charge_action is not None and charge_action.kind == ActionType.CHARGE:
            if not runtime.charge_done_sent:
                self._notify_action_completed(charge_action, self._elapsed(now))
        drive_from = PlannedAction(
            runtime.vehicle_id,
            ActionType.DRIVE_FROM_CHARGER,
            runtime.charge_departure_end - 45 if runtime.charge_departure_end is not None else self._elapsed(now),
            self._elapsed(now),
            runtime.task_idx,
        )
        self._notify_action_started(drive_from)
        self._notify_action_completed(drive_from, self._elapsed(now))
        self._publish_action(runtime.vehicle_id, "wait")
        runtime.command = None
        runtime.action = None
        runtime.action_started_at = None
        runtime.charge_target = None
        runtime.charge_deadline = None
        runtime.charge_departure_end = None
        runtime.charge_done_sent = False
        runtime.charge_started_at = None
        runtime.resume_action = None
        runtime.location = "mission"
        self._replan_requested = True
        self.get_logger().info(f"V{runtime.vehicle_id}: charge cycle completed")

    def _process_vehicle_events(self, runtime: VehicleRuntime, now: float):
        if runtime.command == "wait" and runtime.wait_until is not None and now >= runtime.wait_until:
            self._finish_wait(runtime, now)
        elif runtime.command == "transport" and runtime.status == "transport_done" and runtime.transport_armed:
            self._complete_transport(runtime, now)
        elif runtime.command == "charge_cycle":
            if runtime.status == "docked" and runtime.location != "charger":
                self._on_docked(runtime, now)
            if runtime.status == "parked_after_charge" and runtime.charge_done_sent:
                self._on_parked(runtime, now)
            elif runtime.wait_until is not None and now >= runtime.wait_until:
                self._finish_charger_wait(runtime, now)

    def _process_charging(self, now: float):
        for runtime in self._runtimes.values():
            if (
                runtime.command != "charge_cycle"
                or runtime.status != "docked"
                or runtime.charge_target is None
                or runtime.charge_done_sent
            ):
                continue
            if runtime.soc >= runtime.charge_target:
                self._send_charge_complete(runtime)
                self.get_logger().info(f"V{runtime.vehicle_id}: virtual SoC charge target reached")
                self._replan_requested = True
            elif runtime.charge_deadline is not None and now >= runtime.charge_deadline:
                self._send_charge_complete(runtime)
                self.get_logger().warning(f"V{runtime.vehicle_id}: charge safety timeout reached")
                self._replan_requested = True

    def loop(self):
        now = time.monotonic()
        self._update_soc(now)
        if self._stop_for_empty_battery(now):
            self._record_run_sample()
            return
        for runtime in self._runtimes.values():
            self._process_vehicle_events(runtime, now)
        self._process_charging(now)

        elapsed = self._elapsed(now)
        outage_active = self._outage_at(elapsed) is not None
        if outage_active != self._previous_outage_state:
            self._previous_outage_state = outage_active
            self._replan_requested = True
            self.get_logger().info(
                f"Charger outage {'started' if outage_active else 'ended'} at t={elapsed}s"
            )

        if self._replan_requested:
            self._replan_requested = False
            self._replan(now)
        self._publish_runtime_commands(now)
        self._publish_experiment_info(now)
        self._record_run_sample()
        self._last_loop_at = now


if __name__ == "__main__":
    exp_runner.main()