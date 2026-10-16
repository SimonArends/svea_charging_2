#!/usr/bin/env python3
from __future__ import annotations

import math
import time
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
    CHARGE_MA,
    DEFAULT_INITIAL_SOCS,
    DEFAULT_TASKS,
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
    charge_done_sent: bool = False
    resume_action: PlannedAction | None = None
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


class exp_runner(rx.Node):
    update_rate = rx.Parameter(1.0)
    controller_name = rx.Parameter("STUPID")
    planner_time_limit = rx.Parameter(0.5)
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
        self._runtimes = {
            1: VehicleRuntime(1, self._tasks_a, self._initial_soc_a),
            2: VehicleRuntime(2, self._tasks_b, self._initial_soc_b),
        }
        self._outages = self._parse_outages(str(self.outage_slots))
        self._controller = self._make_controller()
        self._controller.planner_time_limit = float(self.planner_time_limit)
        self._publishers = {
            1: (self.action_pub_a, self.tr_length_pub_a, self.charge_complete_pub_a),
            2: (self.action_pub_b, self.tr_length_pub_b, self.charge_complete_pub_b),
        }
        period = 1.0 / float(self.update_rate)
        self.create_timer(period, self.loop)
        self.get_logger().info(
            f"Experiment runner: {self._controller.name}; "
            f"initial SoC=({self._initial_soc_a:.1f}%, {self._initial_soc_b:.1f}%); "
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
        if float(self.planner_time_limit) <= 0:
            raise ValueError("planner_time_limit must be positive")

    @staticmethod
    def _parse_tasks(value: str, name: str) -> tuple[int, ...]:
        try:
            tasks = tuple(int(item.strip()) for item in value.split(",") if item.strip())
        except ValueError as error:
            raise ValueError(f"{name} must be comma-separated task durations") from error
        if not tasks or any(task not in (90, 180, 270) for task in tasks):
            raise ValueError(f"{name} durations must be 90, 180, or 270 seconds")
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
            )
        return classes[name]()

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
            if runtime.completed_tasks:
                lines.append("    Recent tasks:")
                for task in runtime.completed_tasks[-5:]:
                    lines.append(
                        f"      Task {task.task_idx + 1} ({task.duration}s) "
                        f"{self._format_time(task.start)}-{self._format_time(task.end)} | "
                        f"SoC {task.soc_start:.1f}%->{task.soc_end:.1f}%"
                    )
            else:
                lines.append("    Recent tasks: none")

        if self._controller.name == "STATIC_OPTIMAL":
            plan_actions = self._controller.current_schedule
            plan_label = "Static optimal schedule"
        elif self._controller.name in {"OPTIMIZER", "ORACLE"}:
            plan_actions = self._controller.last_plan
            plan_label = f"{self._controller.name} full remaining plan"
        else:
            plan_actions = self._latest_plan_actions
            plan_label = "Latest immediate plan"
        active_actions = []
        for runtime in self._runtimes.values():
            if runtime.command == "charge_cycle" and runtime.charge_done_sent:
                active_actions.append(
                    PlannedAction(
                        runtime.vehicle_id,
                        ActionType.DRIVE_FROM_CHARGER,
                        elapsed,
                        elapsed + 45,
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
        return "\n".join(lines)

    def _publish_experiment_info(self, now: float):
        self.experiment_info_pub.publish(String(data=self._experiment_info(now)))

    def _outage_at(self, elapsed: int) -> Outage | None:
        return next((outage for outage in self._outages if outage.start <= elapsed < outage.end), None)

    def _knowledge(self, elapsed: int) -> Knowledge:
        observed = tuple(outage for outage in self._outages if outage.start <= elapsed)
        return Knowledge(observed, self._outages)

    def _charge_busy_until(self, now: int, planning_vehicle: int | None = None) -> int:
        busy_until = now
        for runtime in self._runtimes.values():
            if runtime.vehicle_id == planning_vehicle or runtime.command != "charge_cycle":
                continue
            if runtime.charge_done_sent or runtime.status == "parked_after_charge":
                continue
            if runtime.status == "docked" and runtime.charge_target is None:
                continue
            if runtime.status == "docked" and runtime.charge_target is not None:
                remaining = max(
                    0,
                    math.ceil(
                        (runtime.charge_target - runtime.soc)
                        * BATTERY_MAS
                        / (100.0 * CHARGE_MA)
                    ),
                )
                busy_until = max(busy_until, now + remaining)
            elif runtime.status != "parked_after_charge":
                busy_until = max(busy_until, now + 45)
        return busy_until

    def _snapshots(self, now: int) -> tuple[VehicleSnapshot, ...]:
        snapshots = []
        monotonic_now = time.monotonic()
        for runtime in self._runtimes.values():
            if runtime.task_idx >= len(runtime.tasks):
                continue
            busy_until = now
            ready_soc = int(round(runtime.soc * BATTERY_MAS / 100.0))
            ready_task_idx = runtime.task_idx
            ready_location = runtime.location

            if runtime.wait_until is not None:
                remaining = max(0.0, runtime.wait_until - monotonic_now)
                busy_until = now + int(math.ceil(remaining))
                ready_soc -= int(round(remaining * 2000))
            elif runtime.command == "transport" and runtime.action is not None:
                duration = runtime.action.end - runtime.action.start
                finish_at = (runtime.action_started_at or monotonic_now) + duration
                remaining = max(0.0, finish_at - monotonic_now)
                busy_until = now + int(math.ceil(remaining))
                ready_soc -= int(round(remaining * 4500))
                ready_task_idx += 1
                ready_location = "mission"
            elif runtime.command == "charge_cycle":
                if runtime.charge_done_sent:
                    remaining = 45.0
                    ready_soc -= 45 * 4500
                    ready_location = "mission"
                elif runtime.status == "docked" and runtime.charge_target is not None:
                    charge_remaining = max(
                        0.0,
                        (runtime.charge_target - runtime.soc)
                        * BATTERY_MAS
                        / (100.0 * CHARGE_MA),
                    )
                    remaining = charge_remaining + 45.0
                    ready_soc = int(round(runtime.charge_target * BATTERY_MAS / 100.0)) - 45 * 4500
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
                    ready_soc -= int(round(remaining * 4500))
                    ready_location = "charger"
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

    def _begin_wait(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
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
        runtime.resume_action = runtime.action
        runtime.action = action
        runtime.action_started_at = now
        runtime.wait_until = now + max(0, action.end - action.start)
        self._notify_action_started(action)
        if runtime.wait_until <= now:
            self._finish_charger_wait(runtime, now)

    def _finish_charger_wait(self, runtime: VehicleRuntime, now: float):
        action = runtime.action
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
        length = {90: "short", 180: "mid", 270: "long"}[duration]
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
        runtime.command = "charge_cycle"
        runtime.action = action
        runtime.action_started_at = now
        runtime.location = "mission"
        runtime.charge_target = None
        runtime.charge_deadline = None
        runtime.charge_done_sent = False
        runtime.resume_action = None
        self._publish_charge_complete(runtime.vehicle_id, False)
        self._publish_action(runtime.vehicle_id, "charge_cycle")
        self._notify_action_started(action)

    def _start_charging(self, runtime: VehicleRuntime, action: PlannedAction, now: float):
        self._start_experiment(now)
        duration = max(1, action.end - action.start)
        runtime.action = action
        charge_gain_pct = CHARGE_MA * duration * 100.0 / BATTERY_MAS
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
        elif action.kind == ActionType.WAIT:
            if runtime.command == "charge_cycle" and runtime.status == "docked":
                self._begin_charger_wait(runtime, action, now)
            else:
                self._begin_wait(runtime, action, now)

    def _replan(self, now: float):
        elapsed = self._elapsed(now)
        snapshots = self._snapshots(elapsed)
        if not snapshots:
            self._latest_plan_actions = ()
            if self._controller.name != "STATIC_OPTIMAL":
                self._controller.last_plan = ()
            return
        actions = self._controller.decide(
            elapsed,
            snapshots,
            self._charge_busy_until(elapsed),
            self._knowledge(elapsed),
        )
        self._latest_plan_actions = tuple(actions)
        for action in actions:
            runtime = self._runtimes.get(action.vehicle)
            if runtime is None or runtime.status is None:
                continue
            at_charge_decision = runtime.command == "charge_cycle" and runtime.status == "docked" and runtime.charge_target is None
            at_charge_decision = at_charge_decision and runtime.wait_until is None
            if runtime.command is not None and not at_charge_decision:
                continue
            self._execute(action, now)

    def _complete_transport(self, runtime: VehicleRuntime, now: float):
        action = runtime.action
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
        runtime.charge_done_sent = True
        self._publish_charge_complete(runtime.vehicle_id, True)
        if runtime.action is not None and runtime.action.kind == ActionType.CHARGE:
            self._notify_action_completed(runtime.action, self._elapsed(time.monotonic()))
        now = time.monotonic()
        for waiting_runtime in self._runtimes.values():
            if waiting_runtime.command == "wait" and waiting_runtime.wait_until is not None:
                self._finish_wait(waiting_runtime, now)
            elif waiting_runtime.command == "charge_cycle" and waiting_runtime.wait_until is not None:
                self._finish_charger_wait(waiting_runtime, now)

    def _on_docked(self, runtime: VehicleRuntime, now: float):
        runtime.location = "charger"
        previous = runtime.action
        if previous is not None and previous.kind == ActionType.DRIVE_TO_CHARGER:
            self._notify_action_completed(previous, self._elapsed(now))
        runtime.charge_target = None
        runtime.charge_done_sent = False
        self._replan_requested = True

    def _on_parked(self, runtime: VehicleRuntime, now: float):
        charge_action = runtime.action
        if charge_action is not None and charge_action.kind == ActionType.CHARGE:
            if not runtime.charge_done_sent:
                self._notify_action_completed(charge_action, self._elapsed(now))
        drive_from = PlannedAction(
            runtime.vehicle_id,
            ActionType.DRIVE_FROM_CHARGER,
            self._elapsed(runtime.action_started_at or now),
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
        runtime.charge_done_sent = False
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
            if runtime.command != "charge_cycle" or runtime.status != "docked" or runtime.charge_target is None:
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
        self._last_loop_at = now


if __name__ == "__main__":
    exp_runner.main()