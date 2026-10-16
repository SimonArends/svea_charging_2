from __future__ import annotations
import argparse
import csv
import json
from controllers import OracleController, OptimizerController, StaticOptimalController, StupidController
from scenario import make_scenario, soc_pct
from simulator import Simulator, Result

CONTROLLERS = (StupidController, OptimizerController, OracleController, StaticOptimalController)

def scenario_log_fields(s):
    return {
        "initial_soc_pct": json.dumps({f"V{v.id}": round(v.initial_soc_pct, 3) for v in s.vehicles}, separators=(",", ":")),
        "tasks": json.dumps({f"V{v.id}": list(v.tasks) for v in s.vehicles}, separators=(",", ":")),
        "charger_unavailability": json.dumps([[o.start, o.end] for o in s.outages], separators=(",", ":")),
        "exact_task_durations": json.dumps({f"V{v.id}": [s.actual_delivery[(v.id, i)] for i in range(len(v.tasks))] for v in s.vehicles}, separators=(",", ":")),
    }

def print_scenario(s, detailed=False):
    print(f"SCENARIO seed={s.seed}")
    print("Initial SoC: " + ", ".join(f"V{v.id}={v.initial_soc_pct:.1f}%" for v in s.vehicles))
    print("Tasks: " + "; ".join(f"V{v.id}={list(v.tasks)}" for v in s.vehicles))
    print("Outages: " + (", ".join(f"[{o.start},{o.end}]" for o in s.outages) if s.outages else "none"))
    if detailed:
        print("Exact task durations: " + "; ".join(f"V{v.id}={[s.actual_delivery[(v.id, i)] for i in range(len(v.tasks))]}" for v in s.vehicles))

def print_result(r: Result):
    print("\n" + "=" * 70)
    print(r.controller)
    print("=" * 70)
    print(f"Completed:                 {r.completed}")
    print(f"Makespan:                  {r.makespan} s")
    print(f"Total time spent waiting: {r.total_waiting_time} s")
    print(f"Transport time fraction:   {100*r.transport_time_fraction:.1f}%")
    print(f"Charger utilization:       {100*r.charger_utilization:.1f}%")
    print(f"Charging visits:           {r.charging_visits}")
    if r.failure_reason:
        print(f"Failure reason:            {r.failure_reason}")
    if r.actions:
        print("\nActions:")
        for a in sorted(r.actions, key=lambda x: (x.start, x.vehicle, x.end)):
            print(f"t={a.start:4d}-{a.end:4d} V{a.vehicle} {a.kind.value:<20} SoC {soc_pct(a.soc_start):6.2f}% -> {soc_pct(a.soc_end):6.2f}%")

def single(seed, only=None, verbose=True, planner_time_limit=5.20, initial_soc_ranges=None, detailed=False):
    classes = [c for c in CONTROLLERS if only is None or c.name == only]
    controllers = [cls() for cls in classes]
    for c in controllers:
        c.planner_time_limit = planner_time_limit
    s = make_scenario(seed, initial_soc_ranges=initial_soc_ranges)
    print_scenario(s, detailed=detailed)
    results = []
    for c in controllers:
        r = Simulator(s, c, verbose=verbose).run()
        print_result(r)
        results.append(r)
    return results

def sweep(n, planner_time_limit=0.20, initial_soc_ranges=None, detailed=False):
    rows = []
    controllers = [cls() for cls in CONTROLLERS]
    for seed in range(1, n + 1):
        s = make_scenario(seed, initial_soc_ranges=initial_soc_ranges)
        for c in controllers:
            reset = getattr(c, "reset", None)
            if reset is not None:
                reset()
            c.planner_time_limit = planner_time_limit
            r = Simulator(s, c, verbose=False).run()
            row = {"seed": seed, "controller": r.controller, "completed": r.completed, "makespan_s": r.makespan, "total_waiting_time_s": r.total_waiting_time, "transport_time_fraction": r.transport_time_fraction, "charger_utilization": r.charger_utilization, "charging_visits": r.charging_visits, "failure_reason": r.failure_reason or ""}
            if detailed:
                row.update(scenario_log_fields(s))
            rows.append(row)
    with open("results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader(); w.writerows(rows)
    print(f"Wrote {len(rows)} controller results to results.csv")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", type=int, default=1)
    p.add_argument("--controller", choices=[c.name for c in CONTROLLERS])
    p.add_argument("--sweep", action="store_true")
    p.add_argument("--seeds", type=int, default=10)
    p.add_argument("--planner-time-limit", type=float, default=0.50, help="CP-SAT time budget per replanning call in seconds")
    p.add_argument("--soc-range", action="append", nargs=2, type=float, metavar=("MIN", "MAX"), help="Random initial SoC range for one vehicle; repeat once per vehicle")
    p.add_argument("--detailed", action="store_true", help="Include scenario metadata and exact task durations in output")
    a = p.parse_args()
    soc_ranges = tuple(tuple(bounds) for bounds in a.soc_range) if a.soc_range else None
    if a.sweep:
        sweep(a.seeds, a.planner_time_limit, soc_ranges, a.detailed)
    else:
        single(a.scenario, a.controller, planner_time_limit=a.planner_time_limit, initial_soc_ranges=soc_ranges, detailed=a.detailed)
