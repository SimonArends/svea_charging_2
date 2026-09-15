#!/usr/bin/env python3

"""Automatic scheduler. Listens to location, charging need and voltage/current of both
SVEA A and B.

When a SVEA reports that it needs charging, the node computes:
  * eta     - the earliest physical moment it can be at the charger
              (now + its fixed travel-time parameter).
  * start   - the estimated moment it will actually be able to start charging
              (== eta if the charger will be free by then, otherwise the
              estimated moment the other SVEA is expected to be done).
  * finish  - start + the time needed to charge from its current battery
              voltage up to charge_done_voltage, plus a departure margin.

This computation is only *initiated* once per charging request (when need_charging
switches to True), using the scheduling policy: "schedule a svea at the
first moment the charger is available after its expected moment of arrival".
The resulting start/finish estimates are then continuously refined every
control loop using the real battery voltage/current of both SVEAs, and
printed.

Permission is granted early enough for the SVEA to drive to the charger and
arrive when the charger is expected to become free. Permission is therefore
not an indication that charging has started. Actual charging is determined
from the battery current (see charging_detect_current), so the schedule can
self-correct when the real charging current becomes available."""

import time

import svea_core.rosonic as rx

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class automatic_scheduler(rx.Node):

    # --- Parameters -----------------------------------------------------
    # Nominal charging current, used to *project* how long a full charge will
    # take before a SVEA is actually plugged in and a real current reading
    # is available for it.
    charging_current = rx.Parameter(3.0)
    charge_done_voltage = rx.Parameter(12.55)

    # Battery model constants. These must match the values used by the
    # battery_simulator node (or the real battery)
    battery_empty_voltage = rx.Parameter(10.0)
    battery_full_voltage = rx.Parameter(12.6)
    battery_capacity_ah = rx.Parameter(0.9)

    # Extra time added on top of the raw charge-time estimate, so the SVEA
    # has time to disconnect and drive off before the next one arrives.
    departure_margin_s = rx.Parameter(15.0)

    # Travel times from the two possible starting locations to the charger.
    # These are selected from self.location_a / self.location_b ("A" or "B")
    time_a_to_charger = rx.Parameter(77.0)
    time_b_to_charger = rx.Parameter(60.0)

    # Expected battery current while waiting and while driving to the charger.
    idle_current = rx.Parameter(0.9)
    moving_current = rx.Parameter(1.8)

    # Current above this value is treated as actual charging. Permission is
    # granted before arrival, so it must not be used to detect charging.
    charging_detect_current = rx.Parameter(-0.7)

    # Control loop rate [Hz]
    update_rate = rx.Parameter(20.0)
    # Minimum time between two "estimate updated" log lines per SVEA [s]
    print_interval = rx.Parameter(15.0)

    battery_charging_topic_a = rx.Parameter("/svea_a/mavros/battery")
    battery_charging_topic_b = rx.Parameter("/svea_b/mavros/battery")
    need_charging_topic_a = rx.Parameter("/svea_a/mission/need_charging")
    need_charging_topic_b = rx.Parameter("/svea_b/mission/need_charging")

    # --- Publishers -------------------------------------------------
    permission_pub_a = rx.Publisher(Bool, "/svea_a/mission/charge_permission", qos_pubber)
    permission_pub_b = rx.Publisher(Bool, "/svea_b/mission/charge_permission", qos_pubber)

    # --- Subscribers ------------------------------------------------
    @rx.Subscriber(String, "/svea_a/outdoor_stanley/location")
    def _location_a(self, msg: String):
        self.location_a = msg.data
    @rx.Subscriber(String, "/svea_b/outdoor_stanley/location")
    def _location_b(self, msg: String):
        self.location_b = msg.data

    @rx.Subscriber(BatteryState, battery_charging_topic_a, qos_pubber)
    def _battery_charging_cb_a(self, msg: BatteryState):
        self.battery_current_a = float(msg.current)
        self.battery_voltage_a = float(msg.voltage)
    @rx.Subscriber(BatteryState, battery_charging_topic_b, qos_pubber)
    def _battery_charging_cb_b(self, msg: BatteryState):
        self.battery_current_b = float(msg.current)
        self.battery_voltage_b = float(msg.voltage)

    @rx.Subscriber(Bool, need_charging_topic_a, qos_pubber)
    def need_charging_a_cb(self, msg: Bool):
        self.need_charging_a = msg.data
    @rx.Subscriber(Bool, need_charging_topic_b, qos_pubber)
    def need_charging_b_cb(self, msg: Bool):
        self.need_charging_b = msg.data

    def on_startup(self):
        self.location_a = None
        self.location_b = None
        self.battery_current_a = None
        self.battery_voltage_a = None
        self.battery_current_b = None
        self.battery_voltage_b = None
        self.need_charging_a = False
        self.need_charging_b = False
        self.permission_a = False
        self.permission_b = False

        # schedule[letter] is either None (no active request) or a dict:
        #   {"eta": ..., "start": ..., "finish": ..., "free_on_arrival": ...}
        # all timestamps are time.monotonic() values.
        self.schedule = {"a": None, "b": None}
        # Letter of the SVEA currently holding the charger, or None.
        self.active = None
        self._last_log_time = {}

        period = 1.0 / self.update_rate
        self.create_timer(period, self.loop)

    # Helper functions
    @staticmethod
    def other_of(letter):
        return "b" if letter == "a" else "a"

    def travel_time(self, letter):
        # Travel time depends on the SVEA's current location
        # self.location_a / self.location_b contain the location
        # ("A" or "B") of each SVEA.
        location = getattr(self, f"location_{letter}")
        if location == "A":
            return self.time_a_to_charger
        if location == "B":
            return self.time_b_to_charger
        return None

    def get_voltage(self, letter):
        return getattr(self, f"battery_voltage_{letter}")

    def get_current(self, letter):
        return getattr(self, f"battery_current_{letter}")

    def needs_charging(self, letter):
        return getattr(self, f"need_charging_{letter}")

    def compute_charge_duration(self, voltage, current=None):
        """Seconds needed to charge from `voltage` up to charge_done_voltage
        at `current` [A], plus the departure margin. Falls back to the
        nominal charging_current parameter when no valid (positive) current
        reading is available yet, e.g. before the SVEA is actually plugged
        in and charging for real."""
        if voltage is None:
            return None
        if current is None or current <= 0.0:
            current = self.charging_current
        span = self.battery_full_voltage - self.battery_empty_voltage
        if span <= 0.0 or current <= 0.0:
            return None
        delta_v = max(0.0, self.charge_done_voltage - voltage)
        seconds = delta_v * self.battery_capacity_ah * 3600.0 / (span * current)
        return seconds + self.departure_margin_s

    def estimate_start_voltage(self, letter, now):
        """Estimate battery voltage when charging is expected to start.

        Until permission is given, the SVEA is assumed to draw idle_current.
        During the fixed travel time it is assumed to draw moving_current.
        The same linear battery model as compute_charge_duration is used.
        """
        voltage = self.get_voltage(letter)
        sched = self.schedule[letter]
        if voltage is None or sched is None:
            return None

        travel_time = self.travel_time(letter)
        if travel_time is None:
            return None
        travel_s = max(0.0, travel_time)
        wait_s = max(0.0, sched["start"] - sched["eta"])
        consumed_ah = (
            self.idle_current * wait_s + self.moving_current * travel_s
        ) / 3600.0

        span = self.battery_full_voltage - self.battery_empty_voltage
        if self.battery_capacity_ah <= 0.0 or span <= 0.0:
            return None

        voltage_drop = consumed_ah * span / self.battery_capacity_ah
        return max(self.battery_empty_voltage, voltage - voltage_drop)

    def estimate_finish(self, letter, now):
        """Best current estimate (monotonic timestamp) of when `letter` will
        have left the charger - whether it is currently charging, still
        travelling, or waiting to start."""
        sched = self.schedule[letter]
        if sched is None:
            return None
        voltage = self.get_voltage(letter)
        if self.active == letter:
            duration = self.compute_charge_duration(voltage, self.get_current(letter))
            return now + duration if duration is not None else sched["finish"]
        start_voltage = self.estimate_start_voltage(letter, now)
        duration = self.compute_charge_duration(start_voltage, None)
        return sched["start"] + duration if duration is not None else sched["finish"]

    def _fmt(self, t_mono, now_mono=None):
        """Render a monotonic timestamp as a local wall-clock HH:MM:SS string."""
        now_mono = time.monotonic() if now_mono is None else now_mono
        wall = time.time() + (t_mono - now_mono)
        return time.strftime("%H:%M:%S", time.localtime(wall))

    @staticmethod
    def _fmt_voltage(voltage):
        return "unknown" if voltage is None else f"{voltage:.2f}V"

    # --- Scheduling ---------------------------------------------------
    def _start_request(self, letter, now):
        """Runs exactly once per charging request, when need_charging
        changes to True. Decides whether the charger will be free when this
        SVEA arrives, or whether it must queue behind the other one."""
        other = self.other_of(letter)
        travel_time = self.travel_time(letter)
        if travel_time is None:
            self.get_logger().warn(
                f"svea_{letter}: charging requested, but its location is unknown. "
                "Cannot estimate travel time yet."
            )
            return
        eta = now + travel_time

        other_finish = None
        if self.active == other or self.schedule[other] is not None:
            other_finish = self.estimate_finish(other, now)

        if other_finish is not None and other_finish > eta:
            start = other_finish
            free_on_arrival = False
        else:
            start = eta
            free_on_arrival = True

        # Store the schedule before estimating start voltage, because the
        # voltage estimate uses the expected waiting time.
        self.schedule[letter] = {
            "eta": eta,
            "start": start,
            "finish": start,
            "free_on_arrival": free_on_arrival,
        }
        start_voltage = self.estimate_start_voltage(letter, now)
        duration = self.compute_charge_duration(start_voltage, None)
        finish = start + (duration if duration is not None else 0.0)
        self.schedule[letter]["finish"] = finish
        self._last_log_time[letter] = now

        if free_on_arrival:
            self.get_logger().info(
                f"svea_{letter}: charging requested. Charger is free -> "
                f"expecting to arrive in {eta - now:.1f}s and start charging "
                f"immediately, estimated start voltage "
                f"{self._fmt_voltage(self.estimate_start_voltage(letter, now))}, "
                f"finishing around {self._fmt(finish, now)} "
                f"(in {finish - now:.1f}s)."
            )
        else:
            self.get_logger().info(
                f"svea_{letter}: charging requested. Charger occupied by "
                f"svea_{other} -> expecting to arrive in {eta - now:.1f}s, "
                f"then wait until ~{self._fmt(start, now)} to start, "
                f"estimated start voltage {self._fmt_voltage(self.estimate_start_voltage(letter, now))}, "
                f"finishing around {self._fmt(finish, now)}."
            )

    def _refresh_estimate(self, letter, now):
        """Runs every control loop for an already-scheduled request, keeping
        the start/finish estimate up to date using the latest real battery
        voltage (and current, while actually charging)."""
        sched = self.schedule[letter]
        if self.active == letter:
            new_finish = self.estimate_finish(letter, now)
            if new_finish is not None:
                sched["finish"] = new_finish
        else:
            other = self.other_of(letter)
            if not sched["free_on_arrival"]:
                other_finish = self.estimate_finish(other, now)
                if other_finish is not None:
                    sched["start"] = max(sched["eta"], other_finish)
            start_voltage = self.estimate_start_voltage(letter, now)
            duration = self.compute_charge_duration(start_voltage, None)
            if duration is not None:
                sched["finish"] = sched["start"] + duration

        last = self._last_log_time.get(letter, 0.0)
        if now - last >= self.print_interval:
            self._last_log_time[letter] = now
            status = (
                "charging" if self.active == letter
                else "waiting to charge" if now >= sched["eta"]
                else "en route"
            )
            self.get_logger().info(
                f"svea_{letter}: [{status}] updated estimate -> start "
                f"~{self._fmt(sched['start'], now)}, estimated start voltage "
                f"~{self._fmt_voltage(self.estimate_start_voltage(letter, now))}, finish "
                f"~{self._fmt(sched['finish'], now)} "
                f"(in {sched['finish'] - now:.1f}s)."
            )

    def _update_active(self):
        """Determine who is actually charging from battery current.

        Permission is deliberately not used here: it is granted before the
        SVEA reaches the charger. A charging-current reading is the authority.
        """
        charging = []
        for letter in ("a", "b"):
            current = self.get_current(letter)
            if current is not None and current >= self.charging_detect_current:
                charging.append(letter)
        self.active = charging[0] if len(charging) == 1 else None

    def _clear(self, letter):
        if self.schedule[letter] is not None:
            self.get_logger().info(f"svea_{letter}: charging request cleared.")
        self.schedule[letter] = None
        self._last_log_time.pop(letter, None)
        setattr(self, f"permission_{letter}", False)

    def _update_permission(self, letter, now):
        """Grant permission early enough to arrive when the charger is free.

        For a queued SVEA, permission is granted at
        (expected charger-free time - travel time), not when it arrives at the
        charger. Permission is therefore independent of `active`.
        """
        sched = self.schedule[letter]
        travel_time = self.travel_time(letter)
        if travel_time is None:
            setattr(self, f"permission_{letter}", False)
            return
        permission_time = sched["start"] - travel_time
        granted = now >= permission_time
        setattr(self, f"permission_{letter}", granted)

    def loop(self):
        now = time.monotonic()
        self._update_active()

        for letter in ("a", "b"):
            if not self.needs_charging(letter):
                self._clear(letter)
                continue
            if self.schedule[letter] is None:
                self._start_request(letter, now)
            if self.schedule[letter] is not None:
                self._refresh_estimate(letter, now)
                self._update_permission(letter, now)

        self.permission_pub_a.publish(Bool(data=self.permission_a))
        self.permission_pub_b.publish(Bool(data=self.permission_b))


if __name__ == "__main__":
    automatic_scheduler.main()