from __future__ import annotations
from dataclasses import dataclass
from ortools.sat.python import cp_model
from scenario import ActionType, BATTERY_MAS, CHARGE_MA, CHARGE_TARGET_MAS, DRIVE_MA, FROM_CHARGER_S, MIN_SOC_MAS, Outage, PlannedAction, TO_CHARGER_S, WAIT_MA, VehicleSnapshot

@dataclass(frozen=True)
class VehicleInput:
    id: int
    now: int
    soc: int
    task_idx: int
    task_durations: tuple[int, ...]
    location: str
    ready_at: int | None = None

@dataclass(frozen=True)
class PlannerInput:
    now: int
    vehicles: tuple[VehicleInput, ...]
    charger_busy_until: int
    known_outages: tuple[Outage, ...]
    time_limit_s: float = 1.0
    num_search_workers: int = 8

def _horizon(inp: PlannerInput) -> int:
    remaining = sum(sum(v.task_durations[v.task_idx:]) for v in inp.vehicles)
    n = sum(max(0, len(v.task_durations) - v.task_idx) for v in inp.vehicles)
    far = max([inp.now] + [o.end for o in inp.known_outages])
    return max(inp.now + 3600, inp.now + remaining + n * 2200 + 1000, far + 2200)

def plan(inp: PlannerInput) -> list[PlannedAction]:
    vehicles = [v for v in inp.vehicles if v.task_idx < len(v.task_durations)]
    if not vehicles:
        return []
    model = cp_model.CpModel()
    H = _horizon(inp)
    charger_intervals = []
    all_actions = []
    final_ends = []
    visit_bools = []
    post_wait_durations = []
    for v in vehicles:
        prev_end = max(inp.now, v.ready_at if v.ready_at is not None else inp.now)
        soc_in = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_in_{v.task_idx}")
        model.Add(soc_in == v.soc)
        first = True
        for k in range(v.task_idx, len(v.task_durations)):
            dur = int(v.task_durations[k])
            use = model.NewBoolVar(f"v{v.id}_charge_{k}")
            visit_bools.append(use)
            ds = model.NewIntVar(inp.now, H, f"v{v.id}_ds_{k}")
            de = model.NewIntVar(inp.now, H, f"v{v.id}_de_{k}")
            to_s = model.NewIntVar(inp.now, H, f"v{v.id}_to_s_{k}")
            to_e = model.NewIntVar(inp.now, H, f"v{v.id}_to_e_{k}")
            wait_s = model.NewIntVar(inp.now, H, f"v{v.id}_wait_s_{k}")
            wait_e = model.NewIntVar(inp.now, H, f"v{v.id}_wait_e_{k}")
            post_wait_s = model.NewIntVar(inp.now, H, f"v{v.id}_post_wait_s_{k}")
            post_wait_e = model.NewIntVar(inp.now, H, f"v{v.id}_post_wait_e_{k}")
            cs = model.NewIntVar(inp.now, H, f"v{v.id}_cs_{k}")
            ce = model.NewIntVar(inp.now, H, f"v{v.id}_ce_{k}")
            from_s = model.NewIntVar(inp.now, H, f"v{v.id}_from_s_{k}")
            from_e = model.NewIntVar(inp.now, H, f"v{v.id}_from_e_{k}")
            wait_d = model.NewIntVar(0, H - inp.now, f"v{v.id}_wait_d_{k}")
            post_wait_d = model.NewIntVar(0, H - inp.now, f"v{v.id}_post_wait_d_{k}")
            charge_d = model.NewIntVar(0, BATTERY_MAS // CHARGE_MA, f"v{v.id}_charge_d_{k}")
            soc_after_to = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_to_{k}")
            soc_after_pre_wait = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_pre_wait_{k}")
            soc_after_wait = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_wait_{k}")
            soc_after_charge = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_ch_{k}")
            soc_after_from = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_from_{k}")
            soc_out = model.NewIntVar(0, BATTERY_MAS, f"v{v.id}_soc_out_{k}")
            is_first = first
            at_charger = is_first and v.location == "charger"
            first = False
            model.Add(ds >= prev_end)
            model.Add(de == ds + dur)
            model.Add(soc_out == soc_in - dur * DRIVE_MA).OnlyEnforceIf(use.Not())
            if at_charger:
                model.Add(from_s == prev_end).OnlyEnforceIf(use.Not())
                model.Add(from_e == from_s + FROM_CHARGER_S).OnlyEnforceIf(use.Not())
                model.Add(ds == from_e).OnlyEnforceIf(use.Not())
                model.Add(soc_after_from == soc_in - FROM_CHARGER_S * DRIVE_MA).OnlyEnforceIf(use.Not())
                model.Add(soc_after_from >= 0).OnlyEnforceIf(use.Not())
                model.Add(to_s == prev_end).OnlyEnforceIf(use.Not())
                model.Add(to_e == prev_end).OnlyEnforceIf(use.Not())
            else:
                model.Add(to_s == prev_end).OnlyEnforceIf(use.Not())
                model.Add(to_e == prev_end).OnlyEnforceIf(use.Not())
                model.Add(from_s == prev_end).OnlyEnforceIf(use.Not())
                model.Add(from_e == prev_end).OnlyEnforceIf(use.Not())
                model.Add(soc_after_from == soc_in).OnlyEnforceIf(use.Not())
                model.Add(ds == prev_end).OnlyEnforceIf(use.Not())
            model.Add(wait_s == prev_end).OnlyEnforceIf(use.Not())
            model.Add(wait_e == prev_end).OnlyEnforceIf(use.Not())
            model.Add(post_wait_s == prev_end).OnlyEnforceIf(use.Not())
            model.Add(post_wait_e == prev_end).OnlyEnforceIf(use.Not())
            model.Add(cs == prev_end).OnlyEnforceIf(use.Not())
            model.Add(ce == prev_end).OnlyEnforceIf(use.Not())
            model.Add(wait_d == 0).OnlyEnforceIf(use.Not())
            model.Add(post_wait_d == 0).OnlyEnforceIf(use.Not())
            model.Add(charge_d == 0).OnlyEnforceIf(use.Not())
            model.Add(soc_after_pre_wait == soc_in).OnlyEnforceIf(use.Not())
            model.Add(soc_after_to == soc_in).OnlyEnforceIf(use.Not())
            model.Add(soc_after_wait == soc_in).OnlyEnforceIf(use.Not())
            model.Add(soc_after_charge == soc_in).OnlyEnforceIf(use.Not())
            if at_charger:
                model.Add(wait_s == prev_end).OnlyEnforceIf(use)
                model.Add(wait_e == wait_s + wait_d).OnlyEnforceIf(use)
                model.Add(to_s == prev_end).OnlyEnforceIf(use)
                model.Add(to_e == prev_end).OnlyEnforceIf(use)
                model.Add(post_wait_d == 0).OnlyEnforceIf(use)
                model.Add(post_wait_s == prev_end).OnlyEnforceIf(use)
                model.Add(post_wait_e == prev_end).OnlyEnforceIf(use)
                model.Add(soc_after_pre_wait == soc_in).OnlyEnforceIf(use)
                model.Add(soc_after_to == soc_in).OnlyEnforceIf(use)
                model.Add(cs == wait_e).OnlyEnforceIf(use)
                model.Add(soc_after_wait == soc_after_to - wait_d * WAIT_MA).OnlyEnforceIf(use)
            else:
                model.Add(wait_s == prev_end).OnlyEnforceIf(use)
                model.Add(wait_e == wait_s + wait_d).OnlyEnforceIf(use)
                model.Add(to_s == wait_e).OnlyEnforceIf(use)
                model.Add(to_e == to_s + TO_CHARGER_S).OnlyEnforceIf(use)
                model.Add(post_wait_s == to_e).OnlyEnforceIf(use)
                model.Add(post_wait_e == post_wait_s + post_wait_d).OnlyEnforceIf(use)
                model.Add(cs == post_wait_e).OnlyEnforceIf(use)
                model.Add(soc_after_pre_wait == soc_in - wait_d * WAIT_MA).OnlyEnforceIf(use)
                model.Add(soc_after_pre_wait >= MIN_SOC_MAS).OnlyEnforceIf(use)
                model.Add(soc_after_to == soc_after_pre_wait - TO_CHARGER_S * DRIVE_MA).OnlyEnforceIf(use)
                model.Add(soc_after_to >= MIN_SOC_MAS).OnlyEnforceIf(use)
                model.Add(soc_after_wait == soc_after_to - post_wait_d * WAIT_MA).OnlyEnforceIf(use)
            model.Add(ce == cs + charge_d).OnlyEnforceIf(use)
            model.Add(charge_d >= 1).OnlyEnforceIf(use)
            model.Add(cs >= inp.charger_busy_until).OnlyEnforceIf(use)
            model.Add(soc_after_wait >= MIN_SOC_MAS).OnlyEnforceIf(use)
            model.Add(soc_after_charge == soc_after_wait + charge_d * CHARGE_MA).OnlyEnforceIf(use)
            model.Add(soc_after_charge <= BATTERY_MAS).OnlyEnforceIf(use)
            model.Add(from_s == ce).OnlyEnforceIf(use)
            model.Add(from_e == from_s + FROM_CHARGER_S).OnlyEnforceIf(use)
            model.Add(ds == from_e).OnlyEnforceIf(use)
            model.Add(soc_after_from == soc_after_charge - FROM_CHARGER_S * DRIVE_MA).OnlyEnforceIf(use)
            model.Add(soc_after_from >= MIN_SOC_MAS).OnlyEnforceIf(use)
            model.Add(soc_out == soc_after_from - dur * DRIVE_MA).OnlyEnforceIf(use)
            model.Add(soc_after_from >= MIN_SOC_MAS)
            model.Add(soc_out >= CHARGE_TARGET_MAS)
            model.Add(de <= H)

            # The simulator commits one atomic action immediately at each replan.
            if is_first and prev_end == inp.now:
                if at_charger:
                    model.Add(wait_s == inp.now).OnlyEnforceIf(use)
                    model.Add(from_s == inp.now).OnlyEnforceIf(use.Not())
                else:
                    model.Add(wait_s == inp.now).OnlyEnforceIf(use)
                    model.Add(ds == inp.now).OnlyEnforceIf(use.Not())

            crossing_outages = []
            for o in inp.known_outages:
                if o.end <= inp.now:
                    continue
                before = model.NewBoolVar(f"v{v.id}_before_{k}_{o.start}")
                after = model.NewBoolVar(f"v{v.id}_after_{k}_{o.start}")
                model.Add(cs < o.start).OnlyEnforceIf(before)
                model.Add(cs >= o.end).OnlyEnforceIf(after)
                model.AddBoolOr([before, after, use.Not()])
                if not at_charger:
                    crossing = model.NewBoolVar(f"v{v.id}_outage_during_drive_{k}_{o.start}")
                    model.Add(to_s < o.start).OnlyEnforceIf(crossing)
                    model.Add(to_e >= o.start + 1).OnlyEnforceIf(crossing)
                    model.Add(to_e < o.end).OnlyEnforceIf(crossing)
                    crossing_outages.append(crossing)
            if not at_charger:
                model.Add(post_wait_d <= (H - inp.now) * sum(crossing_outages))
            interval = model.NewOptionalIntervalVar(cs, charge_d, ce, use, f"v{v.id}_charge_interval_{k}")
            charger_intervals.append(interval)
            all_actions.append((
                v, k, use, to_s, to_e, wait_s, wait_e, post_wait_s,
                post_wait_e, cs, ce, from_s, from_e, ds, de, charge_d,
                post_wait_d, soc_in, soc_after_pre_wait, soc_after_to,
                soc_after_wait, soc_after_charge, soc_after_from, soc_out,
            ))
            post_wait_durations.append(post_wait_d)
            final_ends.append(de)
            soc_in = soc_out
            prev_end = de
    if inp.charger_busy_until > inp.now:
        fixed = model.NewIntervalVar(inp.now, inp.charger_busy_until - inp.now, inp.charger_busy_until, "existing_charger_use")
        charger_intervals.append(fixed)
    if charger_intervals:
        model.AddNoOverlap(charger_intervals)
    makespan = model.NewIntVar(inp.now, H, "makespan")
    model.AddMaxEquality(makespan, final_ends)
    max_post_wait = (H - inp.now) * len(post_wait_durations)
    visit_weight = max_post_wait + 1
    makespan_weight = len(visit_bools) * visit_weight + max_post_wait + 1
    model.Minimize(makespan * makespan_weight + sum(visit_bools) * visit_weight + sum(post_wait_durations))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = inp.time_limit_s
    solver.parameters.num_search_workers = inp.num_search_workers
    status = solver.Solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return []
    out = []
    def soc_pct(variable) -> float:
        return 100.0 * solver.Value(variable) / BATTERY_MAS

    def add_action(vehicle, kind, start, end, task_idx, soc_start, soc_end):
        out.append(PlannedAction(
            vehicle,
            kind,
            solver.Value(start),
            solver.Value(end),
            task_idx,
            soc_pct(soc_start),
            soc_pct(soc_end),
        ))

    for (
        v, k, use, to_s, to_e, wait_s, wait_e, post_wait_s, post_wait_e,
        cs, ce, from_s, from_e, ds, de, charge_d, post_wait_d, soc_in,
        soc_after_pre_wait, soc_after_to, soc_after_wait, soc_after_charge,
        soc_after_from, soc_out,
    ) in all_actions:
        use_b = solver.Value(use)
        if use_b:
            if not (v.location == "charger" and k == v.task_idx):
                if solver.Value(wait_e) > solver.Value(wait_s):
                    wait_soc_end = soc_after_wait if v.location == "charger" and k == v.task_idx else soc_after_pre_wait
                    add_action(v.id, ActionType.WAIT, wait_s, wait_e, k, soc_in, wait_soc_end)
                add_action(v.id, ActionType.DRIVE_TO_CHARGER, to_s, to_e, k, soc_after_pre_wait, soc_after_to)
                if solver.Value(post_wait_e) > solver.Value(post_wait_s):
                    add_action(v.id, ActionType.WAIT, post_wait_s, post_wait_e, k, soc_after_to, soc_after_wait)
            elif solver.Value(wait_e) > solver.Value(wait_s):
                add_action(v.id, ActionType.WAIT, wait_s, wait_e, k, soc_in, soc_after_wait)
            add_action(v.id, ActionType.CHARGE, cs, ce, k, soc_after_wait, soc_after_charge)
            add_action(v.id, ActionType.DRIVE_FROM_CHARGER, from_s, from_e, k, soc_after_charge, soc_after_from)
        elif v.location == "charger" and k == v.task_idx:
            add_action(v.id, ActionType.DRIVE_FROM_CHARGER, from_s, from_e, k, soc_in, soc_after_from)
        delivery_soc_start = soc_after_from if use_b or (v.location == "charger" and k == v.task_idx) else soc_in
        add_action(v.id, ActionType.DELIVERY, ds, de, k, delivery_soc_start, soc_out)
    return sorted(out, key=lambda a: (a.start, a.vehicle, a.kind.value))

def first_actions(actions: list[PlannedAction], now: int) -> list[PlannedAction]:
    result = {}
    for a in actions:
        if a.start != now or a.end <= now:
            continue
        if a.vehicle not in result or (a.start, a.end) < (result[a.vehicle].start, result[a.vehicle].end):
            result[a.vehicle] = a
    return [result[k] for k in sorted(result)]

def planner_inputs(now: int, vehicles: tuple[VehicleSnapshot, ...], charger_busy_until: int, outages: tuple[Outage, ...]) -> PlannerInput:
    inputs = []
    for v in vehicles:
        task_idx = v.ready_task_idx if v.busy_until > now and v.ready_task_idx is not None else v.task_idx
        if task_idx >= len(v.tasks):
            continue
        soc = v.ready_soc if v.busy_until > now and v.ready_soc is not None else v.soc
        location = v.ready_location if v.busy_until > now and v.ready_location is not None else v.location
        ready_at = max(now, v.busy_until)
        inputs.append(VehicleInput(v.id, now, soc, task_idx, v.tasks, location, ready_at))
    return PlannerInput(now, tuple(inputs), charger_busy_until, outages)
