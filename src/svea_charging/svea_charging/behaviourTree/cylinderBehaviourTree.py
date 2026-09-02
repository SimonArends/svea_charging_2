from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from svea_charging.third_party.btree.btree import (
    ActionNode,
    Fallback,
    NodeStatus,
    Sequence,
)



@dataclass
class MissionBlackboard:
    # --- Battery Data ---
    battery_current: float = -1.0
    battery_voltage: float | None = None
    communication_ok: bool = True
    charging_active: bool = False
    charging_error: bool = False
    charger_visible: bool = False
    
    # --- Navigation and Docking Parameters ---
    dist_to_switching_point: float | None = None
    aruco_distance: float | None = None
    
    # --- Cylinders Parameters ---
    cylinders_visible: bool = False
    cylinder_distance: float | None = None

    # --- Docking State Latch ---
    docking_engaged: bool = False

    # --- Thresholds and Operational Parameters ---
    gps_switch_distance_m: float = 100.5
    cylinder_switch_distance_m: float = 1.2
    docking_exit_distance_m: float = 2.75
    cylinder_arm_deploy_distance_m: float = 0.2
    charge_start_voltage: float = 12.2
    charge_done_voltage: float = 12.6
    charge_voltage_confirm_s: float = 3.0
    charge_voltage_reached_at: float | None = None
    
    # --- Outputs ---
    active_controller: str = "stanley"
    mission_phase: str = "approach"
    last_tree_status: str = NodeStatus.RUNNING
    last_running_node: str = "startup"


class ChargingMissionTree:
    """
    Minimal behaviour tree for controller handoff during charging approach.

    The tree is intentionally small so it can become the logic owner today and
    grow with new leaves later. It currently owns:
    - controller selection: Stanley -> cylinder docking
    - proximity-based switching
    - simple docking completion
    - charge completion wait
    - communication guard
    """
    def __init__(
        self,
        blackboard: MissionBlackboard,
        set_charging_arm: Callable[[bool], None],
    ):
        self.bb = blackboard
        self.set_charging_arm = set_charging_arm
        approach_phase = Fallback(
            ActionNode(self.is_near_docking_zone, "is_near_docking_zone"),
            ActionNode(self.run_stanley_approach, "run_stanley_approach"),
            name="approach_phase",
        )
        docking_phase = Fallback(
            ActionNode(self.is_docked, "is_docked"),
            ActionNode(self.run_cylinder_docking, "run_cylinder_docking"),
            name="docking_phase",
        )
        charge_phase = Sequence(
            ActionNode(self.needs_charging, "needs_charging"),
            approach_phase,
            docking_phase,
            ActionNode(self.wait_until_charged, "wait_until_charged"),
            name="charge_phase",
        )

        self.tree = Sequence(
            Fallback(
                ActionNode(self.communication_ok, "communication_ok"),
                ActionNode(self.handle_communication_error, "handle_communication_error"),
                name="communication_guard",
            ),
            Fallback(
                ActionNode(self.is_charged, "is_charged"),
                charge_phase,
                name="decision_phase",
            ),
            ActionNode(self.exit_station, "exit_station"),
            name="charging_mission",
        )

    def tick(self) -> str:
        status = self.tree.run()
        self.bb.last_tree_status = status
        self.bb.last_running_node = self._current_running_node_name()
        return status

    @property
    def state(self) -> str:
        return self.bb.last_running_node

    def communication_ok(self) -> str:
        return NodeStatus.SUCCESS if self.bb.communication_ok else NodeStatus.FAILURE

    def handle_communication_error(self) -> str:
        self.set_charging_arm(False)
        self.bb.active_controller = "idle"
        self.bb.mission_phase = "communication_error"
        return NodeStatus.FAILURE

    def is_near_docking_zone(self) -> str:
        if self.bb.docking_engaged:
            self.bb.mission_phase = "docking"
            return NodeStatus.SUCCESS

        gps_dist = self.bb.dist_to_switching_point
        # cyl_dist = self.bb.cylinder_distance

        if (
            gps_dist is not None and gps_dist <= self.bb.gps_switch_distance_m
            # and cyl_dist is not None and cyl_dist <= self.bb.cylinder_switch_distance_m
        ):
            self.bb.docking_engaged = True
            self.bb.mission_phase = "docking"
            return NodeStatus.SUCCESS

        return NodeStatus.FAILURE

    def run_stanley_approach(self) -> str:
        self.bb.active_controller = "stanley"
        self.bb.mission_phase = "approach"
        return NodeStatus.RUNNING

    def is_docked(self) -> str:
        if self.bb.battery_current > 0.0:
            self.bb.active_controller = "idle"
            self.bb.mission_phase = "docked"
            self.bb.charging_active = True
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def run_cylinder_docking(self) -> str:
        self.bb.active_controller = "cylinder_docking"
        self.bb.mission_phase = "docking"

        if self.bb.cylinders_visible:
            if (
                self.bb.cylinder_distance is not None
                and self.bb.cylinder_distance <= self.bb.cylinder_arm_deploy_distance_m
            ):
                self.set_charging_arm(True)
            else:
                self.set_charging_arm(False)
            return NodeStatus.RUNNING

        self.set_charging_arm(False)
        return NodeStatus.RUNNING

    def _current_running_node_name(self) -> str:
        current = self._deepest_running_node(self.tree)
        if current is None:
            return self.bb.mission_phase
        return getattr(current, "name", self.bb.mission_phase)

    def _deepest_running_node(self, node):
        current = getattr(node, "currentRunningNode", None)
        if current is None:
            return None
        if current is node:
            return current
        deeper = self._deepest_running_node(current)
        return deeper if deeper is not None else current

    def needs_charging(self) -> str:
        if (
            self.bb.charging_active
            or self.bb.battery_voltage is None
            or self.bb.battery_voltage < self.bb.charge_start_voltage
        ):
            return NodeStatus.SUCCESS
        self.bb.active_controller = "idle"
        self.bb.mission_phase = "charge_not_needed"
        return NodeStatus.FAILURE

    def is_charged(self) -> str:
        voltage = self.bb.battery_voltage
        if voltage is None or voltage < self.bb.charge_done_voltage:
            self.bb.charge_voltage_reached_at = None
            return NodeStatus.FAILURE

        if self.bb.charge_voltage_reached_at is None:
            self.bb.charge_voltage_reached_at = time.monotonic()
            return NodeStatus.FAILURE

        if (
            time.monotonic() - self.bb.charge_voltage_reached_at
            < self.bb.charge_voltage_confirm_s
        ):
            return NodeStatus.FAILURE

        if voltage >= self.bb.charge_done_voltage:
            self.bb.active_controller = "idle"
            self.bb.mission_phase = "charged"
            self.bb.charging_active = False
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def wait_until_charged(self) -> str:
        if self.is_charged() == NodeStatus.SUCCESS:
            return NodeStatus.SUCCESS
        self.bb.active_controller = "idle"
        self.bb.mission_phase = "charging"
        self.bb.charging_active = True
        return NodeStatus.RUNNING

    def exit_station(self) -> str:
        self.set_charging_arm(False)
        self.bb.active_controller = "stanley"
        self.bb.mission_phase = "exit_station"
        self.bb.charge_voltage_reached_at = None
        self.bb.docking_engaged = False
        return NodeStatus.RUNNING
