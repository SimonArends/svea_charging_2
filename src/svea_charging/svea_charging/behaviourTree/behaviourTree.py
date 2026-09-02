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
    battery_level: float = 20.0
    battery_current: float = -1.0
    battery_voltage: float = 12.0
    communication_ok: bool = True
    charger_visible: bool = False
    charging_active: bool = False
    charging_error: bool = False
    dist_to_station: float | None = None
    aruco_distance: float | None = None
    switch_distance_m: float = 2.8
    dock_distance_m: float = 1.6
    charge_done_level: float = 95.0
    charge_demo_duration_s: float = 10.0
    charge_started_at: float | None = None
    charge_demo_complete: bool = False
    active_controller: str = "stanley"
    mission_phase: str = "approach"
    last_tree_status: str = NodeStatus.RUNNING
    last_running_node: str = "startup"


class ChargingMissionTree:
    """
    Minimal behaviour tree for controller handoff during charging approach.

    The tree is intentionally small so it can become the logic owner today and
    grow with new leaves later. It currently owns:
    - controller selection: Stanley -> MPC
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
            ActionNode(self.run_line_follower_docking, "run_line_follower_docking"),
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
        self.bb.active_controller = "idle"
        self.bb.mission_phase = "communication_error"
        return NodeStatus.FAILURE

    def is_near_docking_zone(self) -> str:
        distance = self.bb.aruco_distance
        if distance is None or distance > self.bb.switch_distance_m:
            return NodeStatus.FAILURE
        if distance <= self.bb.switch_distance_m:
            self.bb.mission_phase = "docking"
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def run_stanley_approach(self) -> str:
        self.bb.active_controller = "stanley"
        self.bb.mission_phase = "approach"
        return NodeStatus.RUNNING

    def is_docked(self) -> str:
        aruco_distance = self.bb.aruco_distance
        if aruco_distance is None:
            return NodeStatus.FAILURE
        if self.bb.battery_current > -1.0:
            self.bb.active_controller = "idle"
            self.bb.mission_phase = "docked"
            self.bb.charging_active = True
            if self.bb.charge_started_at is None:
                self.bb.charge_started_at = time.monotonic()
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def run_mpc_docking(self) -> str:
        self.bb.active_controller = "mpc"
        self.bb.mission_phase = "docking"

        if self.bb.charger_visible and self.bb.line_visible:
            if self.bb.aruco_distance is not None and self.bb.aruco_distance <= 2.0:
                self.set_charging_arm(True)
            return NodeStatus.RUNNING

        return NodeStatus.FAILURE

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
        if self.bb.battery_level < self.bb.charge_done_level:
            return NodeStatus.SUCCESS
        self.bb.active_controller = "idle"
        self.bb.mission_phase = "charge_not_needed"
        return NodeStatus.FAILURE

    def is_charged(self) -> str:
        if self.bb.battery_level >= self.bb.charge_done_level:
            self.bb.active_controller = "idle"
            self.bb.mission_phase = "charged"
            self.bb.charging_active = False
            self.bb.charge_demo_complete = True
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def wait_until_charged(self) -> str:
        if self.is_charged() == NodeStatus.SUCCESS:
            return NodeStatus.SUCCESS
        self.bb.active_controller = "idle"
        self.bb.mission_phase = "charging"
        self.bb.charging_active = True
        if self.bb.charge_started_at is None:
            self.bb.charge_started_at = time.monotonic()
        elapsed_s = time.monotonic() - self.bb.charge_started_at
        if elapsed_s >= self.bb.charge_demo_duration_s:
            self.bb.battery_level = self.bb.charge_done_level
            self.bb.charge_demo_complete = True
            return self.is_charged()
        return NodeStatus.RUNNING

    def exit_station(self) -> str:
        self.set_charging_arm(False)
        self.bb.active_controller = "stanley"
        self.bb.mission_phase = "exit_station"
        self.bb.charge_started_at = None
        return NodeStatus.RUNNING
