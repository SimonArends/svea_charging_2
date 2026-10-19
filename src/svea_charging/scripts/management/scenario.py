from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
import random

BATTERY_MAS = 800 * 3600
MIN_SOC_MAS = BATTERY_MAS // 10 # 10% floor used by STUPID.
CHARGE_TARGET_MAS = MIN_SOC_MAS + 1 * BATTERY_MAS // 100 # 11% margin used by STUPID.
DRIVE_MA = 4500
CHARGE_MA = 3000
WAIT_MA = 2000
TO_CHARGER_S = 45
FROM_CHARGER_S = 45
DEFAULT_INITIAL_SOCS = (45, 45)
DEFAULT_TASKS = ((180, 90), (90, 90, 90))

class ActionType(str, Enum):
    DELIVERY = "DELIVERY"
    DRIVE_TO_CHARGER = "DRIVE_TO_CHARGER"
    CHARGE = "CHARGE"
    DRIVE_FROM_CHARGER = "DRIVE_FROM_CHARGER"
    WAIT = "WAIT"

@dataclass(frozen=True)
class Outage:
    start: int
    end: int

@dataclass(frozen=True)
class VehicleSpec:
    id: int
    initial_soc_pct: float
    tasks: tuple[int, ...]

@dataclass(frozen=True)
class Scenario:
    seed: int
    vehicles: tuple[VehicleSpec, ...]
    outages: tuple[Outage, ...]
    actual_delivery: dict[tuple[int, int], int]

@dataclass
class VehicleState:
    id: int
    soc: int
    tasks: tuple[int, ...]
    task_idx: int = 0
    location: str = "mission"
    busy_until: int = 0
    delivery_time: int = 0
    completion_times: list[int] | None = None
    def __post_init__(self):
        if self.completion_times is None:
            self.completion_times = []

@dataclass(frozen=True)
class VehicleSnapshot:
    id: int
    soc: int
    tasks: tuple[int, ...]
    task_idx: int
    location: str
    busy_until: int
    ready_soc: int | None = None
    ready_task_idx: int | None = None
    ready_location: str | None = None
    charging: bool = False
    charger_trip_started_at: int | None = None

@dataclass(frozen=True)
class PlannedAction:
    vehicle: int
    kind: ActionType
    start: int
    end: int
    task_idx: int | None = None
    soc_start_pct: float | None = None
    soc_end_pct: float | None = None

@dataclass(frozen=True)
class ExecutedAction:
    vehicle: int
    kind: ActionType
    start: int
    end: int
    soc_start: int
    soc_end: int
    task_idx: int | None = None

@dataclass(frozen=True)
class Knowledge:
    observed_outages: tuple[Outage, ...]
    all_outages: tuple[Outage, ...]

def soc_pct(soc_mas: int) -> float:
    return 100.0 * soc_mas / BATTERY_MAS

def soc_cost(ma: int, seconds: int) -> int:
    return ma * seconds

def required_drive_soc(task_seconds: int) -> int:
    return soc_cost(DRIVE_MA, task_seconds)

def can_complete_delivery(soc: int, duration: int) -> bool:
    return soc >= required_drive_soc(duration)

def outage_at(t: int, outages: tuple[Outage, ...] | list[Outage]) -> Outage | None:
    for o in outages:
        if o.start <= t < o.end:
            return o
    return None

def make_scenario(seed: int, initial_socs=DEFAULT_INITIAL_SOCS, tasks=DEFAULT_TASKS, outage_mean=600.0, outage_min=10, outage_max=600, outage_horizon=2000, delivery_sigma=0.0, initial_soc_ranges=None) -> Scenario:
    rng = random.Random(seed)
    if initial_soc_ranges is None:
        resolved_socs = initial_socs
    else:
        if len(initial_soc_ranges) != len(tasks):
            raise ValueError("Provide one initial SoC range for each vehicle")
        soc_rng = random.Random(seed ^ 0x51C0A11)
        resolved_socs = []
        for vehicle_id, (low, high) in enumerate(initial_soc_ranges, start=1):
            if not (math.isfinite(low) and math.isfinite(high) and 0 <= low <= high <= 100):
                raise ValueError(f"Invalid initial SoC range for V{vehicle_id}: {(low, high)}")
            integer_low = math.ceil(low)
            integer_high = math.floor(high)
            if integer_low > integer_high:
                raise ValueError(f"Initial SoC range for V{vehicle_id} contains no whole percent")
            resolved_socs.append(soc_rng.randint(integer_low, integer_high))
    if len(resolved_socs) != len(tasks):
        raise ValueError("Provide one initial SoC value for each vehicle")
    vehicles = tuple(VehicleSpec(i + 1, float(resolved_socs[i]), tuple(tasks[i])) for i in range(len(tasks)))
    outages = []
    t = int(round(rng.expovariate(1.0 / outage_mean)))
    while t < outage_horizon:
        duration = rng.randint(outage_min, outage_max)
        end = min(outage_horizon, t + duration)
        outages.append(Outage(t, end))
        t = end + max(1, int(round(rng.expovariate(1.0 / outage_mean))))
    actual = {}
    for v in vehicles:
        for k, nominal in enumerate(v.tasks):
            actual[(v.id, k)] = max(1, int(round(rng.gauss(nominal, delivery_sigma * nominal))))
    return Scenario(seed, vehicles, tuple(outages), actual)

def initial_states(scenario: Scenario) -> dict[int, VehicleState]:
    return {v.id: VehicleState(v.id, int(round(v.initial_soc_pct * BATTERY_MAS / 100.0)), v.tasks) for v in scenario.vehicles}
