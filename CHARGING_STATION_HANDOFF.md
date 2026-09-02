# Charging station integration — handoff

## Goal for this branch

Two things, both scoped to `outdoor_bt_charging.launch.py` / the outdoor BT charging demo:

1. **Selectable preset routes.** ~~Right now there is exactly one hardcoded
   route.~~ Done — see "Route presets" below. Route files now live under
   `params/routes/`, and `outdoor_bt_charging.launch.py` takes a
   `route_preset` arg (default `"parking_lot"`) that looks up both the route
   file and its `aruco_marker_id` from `ROUTE_PRESETS` in the launch file.
   Add a new preset there when you record a new route/station.
2. **Actually integrate the physical charging station** at the end of the
   route — the BT already has a state machine for it (see below), but the
   final hardware hookup / verification with a real charging station has not
   been done. That's the main point of this branch.

This file is a snapshot of everything that was learned/fixed in the previous
conversation so you don't have to re-derive it. `OUTDOOR_BT_HANDOFF.md` in the
repo root is an **older, superseded** handoff from earlier in that same
debugging session — this file replaces it as the source of truth.

## Architecture (current, working)

BT-mode data flow, all nodes started by `outdoor_bt_charging.launch.py`:

```
outdoor_stanley.py  --publishes-->  stanley/cmd_steering_rad, stanley/cmd_velocity_mps
line_follower.py    --publishes-->  line_follower/cmd_steering_rad, line_follower/cmd_velocity_mps
bt_runner.py         --publishes--> mission/active_controller ("stanley" | "line_follower" | "idle")
control_mux.py       --reads-->     mission/active_controller + both cmd_* topics
                      --owns-->     ActuationInterface (the only thing allowed to touch the LLI)
```

`control_mux.py` is the **only** node that instantiates `ActuationInterface`
and sends to `mavros/manual_control/send`. `outdoor_stanley.py` used to also
own an `ActuationInterface` for a "direct" standalone mode — that mode was
**removed entirely** this session because it silently collided with
`control_mux` on the same topic (both publish at 20Hz; the observer's node,
even disabled, kept publishing neutral messages that raced with the real
commands). If you ever see a node other than `control_mux.py` importing
`ActuationInterface`, that's a bug — don't reintroduce it.

## Controller-selection logic (the "switch" you're asking about)

Lives in `svea_charging/svea_charging/behaviourTree/behaviourTree.py`
(`ChargingMissionTree`, ticked at 20Hz from `bt_runner.py`). It already
implements the full lifecycle:

- `needs_charging` — battery voltage gate (see `bt_charge_start_voltage`
  below) or exit early if not needed.
- `approach_phase` — runs Stanley (`active_controller = "stanley"`) until
  `aruco_distance <= switch_distance_m` (default 2.2m), then hands off.
- `docking_phase` — runs `line_follower.py` (`active_controller =
  "line_follower"`), and arms the physical charging arm
  (`/charging_arm` bool topic → `control_mux` → `ActuationInterface.send_xtr`
  → `aux4` channel) once `aruco_distance <= 0.91m`.
- `is_docked` — currently just checks `battery_current > 0.0` (i.e. "are we
  actually receiving charge current"). **This is the part most likely to need
  real hardware verification/tuning on this branch** — it was never tested
  against a real charging station, only the BT logic and the
  Stanley→line_follower handoff were validated outdoors.
- `wait_until_charged` / `is_charged` — waits for `battery_voltage >=
  charge_done_voltage` sustained for `charge_voltage_confirm_s`, then exits
  back to Stanley (`exit_station`).

So: the state machine and the switch are already there and wired end to end.
What's untested is the actual electrical/physical docking with a real station
(does `aux4`/xtr1 actually drive the right relay/arm on your hardware, does
`battery_current`/`battery_voltage` from `/self/mavros/battery` read
correctly while charging, etc).

Relevant BT params (passed from `outdoor_bt_charging.launch.py`):

```
bt_dock_distance_m: float = 0.617
bt_switch_distance_m: float = 2.2
bt_docking_exit_distance_m: float = 2.75
bt_charge_start_voltage: float = 12.4   # raised from 12.2 this session — see below
bt_charge_done_voltage: float = 12.6
bt_charge_voltage_confirm_s: float = 3.0
```

`bt_charge_start_voltage` was deliberately raised from `12.2` to `12.4` this
session because the battery pack (looks like 3S LiPo given `charge_done=12.6`
= full 3S) kept sitting above the old threshold between test runs, so
`needs_charging` returned `FAILURE` and the whole mission sat at
`active_controller="idle"` — looked like a bug, wasn't. If you change battery
packs or charging behavior, re-check this threshold against your actual
resting/charged voltage curve.

## ArUco marker targeting — real bug fixed this session

`aruco_camera_test.py` used to publish `aruco/distance_m` from **whichever
detected marker was last in the loop**, regardless of ID — so any stray
marker in view within `switch_distance_m` could trigger the docking handoff.
Fixed: it now only updates the published distance when the detected marker ID
matches the `marker_id` parameter (launch default now `11`, matching the
physical marker actually printed/used — was `13` before, wrong ID). If you
add a second route/station with a different physical marker, remember to
pass a different `aruco_marker_id` per preset.

## Stanley/control tuning (stable as of last test — do not re-tune blindly)

`svea_charging/svea_charging/controllers/stanleyController.py` module-level
constants:

```python
k = 0.28                          # Stanley cross-track gain
Kp = 0.6; Ki = 0.2; Kd = 0.01      # velocity PID
max_steer = radians(40.0)         # physical steering limit
max_steer_rate = radians(40.0)    # steering slew rate limit
max_velocity = 0.4                # PID output hard ceiling [m/s]
max_velocity_rate = 0.5           # velocity command slew rate [m/s^2]
```

Launch defaults (`outdoor_bt_charging.launch.py`):

```python
stanley_target_velocity: float = 0.28
stanley_turn_velocity: float = 0.25
stanley_max_steering_rad: float = 0.35
control_mux_timeout_s: float = 1.0
```

This combination was reached after ~20 test iterations chasing: oscillation
(too-high `k`), a permanent one-directional bias (too-low `k`), a velocity
control loop that was fully defeated by an outer clamp (removed — see
`_limit_command` in `outdoor_stanley.py`, it intentionally does **not**
re-clamp velocity to `target_velocity` anymore, only steering), PID integral
windup after a safety stop (fixed via `StanleyController.reset_pid()`,
called from `outdoor_stanley._stop()` and on runtime re-activation), and an
instantaneous velocity-command jump to the PID ceiling on activation (fixed
via the velocity slew-rate limiter, same mechanism as steering already had).
**If the car starts oscillating or drifting again, check whether `k`,
`max_velocity`, or `target_velocity` got changed before assuming it's a new
bug** — this exact combination was validated over a full successful run
end-to-end.

Also fixed this session, unrelated to gains: the wheel encoder was physically
wired backwards (fixed in hardware, not software) — before that fix, velocity
feedback was frequently near-zero garbage even while the car moved at real
speed, which was destabilizing the whole PID loop. If a future car/chassis
shows the same symptom (`odometry/global` twist.x reads ~0 while the car is
visibly moving at constant speed), check encoder wiring/polarity first.

## Route data — how the current route was built, and how to build a new one

`params/routes/to_parking_lot.yaml`, param `map_waypoints`, is a flat JSON-ish string
of `[x, y]` pairs in the outdoor global EKF's `map` frame (same frame as
`/self/odometry/global`). `outdoor_stanley.py` fits a cubic spline
(`third_party/PythonRobotics/PathPlanning/CubicSpline/cubic_spline_planner.py`)
through all of them and Stanley-tracks that spline — **not** straight lines
between waypoints.

Important gotchas learned the hard way, if you build a new preset route:

1. **The spline is a global fit, not per-segment.** `CubicSpline1D` solves a
   single tridiagonal system across every waypoint (natural boundary
   conditions only at the very first/last point). A sharp turn later in the
   list can bleed curvature backward into an earlier "straight" section if
   that section is under-constrained (too few waypoints). Don't rely on
   "start point + end point only" for a straight segment that's immediately
   followed by a turn — put waypoints every ~1m along it so the spline is
   pinned straight right up to the turn.
2. **Raw recorded GPS waypoints are not perfectly collinear**, even on a
   genuinely straight run — up to ~0.2-0.4m of perpendicular jitter was
   observed on this hardware. If you're recording a straight stretch, either
   (a) rectify it afterward — take the two trusted endpoints and regenerate
   points exactly on the line between them at fixed spacing (this is what
   was done here), or (b) don't trust a raw recording for "should be
   straight" segments at all.
3. **`minimum_turning_radius` (default 0.40m) rejects the whole route** at
   startup if peak spline curvature exceeds `1/minimum_turning_radius`
   anywhere (`outdoor_stanley._initialize_path`, logs `route curvature ...
   exceeds limit` and the node just sits inert — easy to mistake for "not
   reacting to enabled=true"). A human steering an RC car by hand around a
   corner can produce a tighter path than the car can later track
   autonomously — check curvature with the spline planner directly before
   deploying a newly recorded route:
   ```python
   from third_party.PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner
   cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.05)
   peak = max(abs(v) for v in ck)   # must be <= 1/minimum_turning_radius
   ```
4. **For a real turn/corner between two straight legs**, don't hand-pick
   points from a wobbly manual recording — measure the two straight legs'
   headings, then generate a circular fillet arc (tangent to both legs, pick
   a radius comfortably above `minimum_turning_radius`, e.g. 3-3.5m) and
   splice it in. This is what produced the current route's turn and it
   passed curvature checks cleanly on the first try, unlike hand-extracted
   points from the raw recording.
5. **Anchor points (start, turn point, goal point) are most reliable as
   single-shot stationary reads**, not extracted from a continuous drive:
   ```bash
   ros2 topic echo --once /self/odometry/global
   ros2 topic echo --once /self/gps/carrier_solution   # must read 2 (fixed)
   ```
   Check the position covariance in the echoed message too — a good fixed
   read on this hardware has covariance ~0.01-0.1 on x/y. A covariance in the
   hundreds of thousands means the EKF hadn't converged yet (seen once when
   echoing immediately after node startup) — discard and re-read.
6. The live route currently ends at a **registered goal point**
   (`[51.2689, -34.9169]` in map frame) partway along what was actually a
   longer manual recording — the recording continued into a much sharper
   loop/hook after that point which both failed the curvature check and
   wasn't part of the intended route. If you reuse that raw bag
   (`record_waypoints`) for anything, don't extract past that goal point
   without deliberately re-checking curvature.

### Route presets (implemented this session)

`outdoor_bt_charging.launch.py` now has a `ROUTE_PRESETS` dict at module
level mapping a preset name (`route_preset: str` arg, default
`"parking_lot"`) to `{route_config, aruco_marker_id}`. Route files live under
`params/routes/<name>.yaml`. To add a new route/station:

1. Record and rectify the route as described above, save it as
   `params/routes/<name>.yaml`.
2. Add an entry to `ROUTE_PRESETS` in `outdoor_bt_charging.launch.py` with
   that file and the marker ID physically mounted on that station.
3. `route_config` (explicit path) and `aruco_marker_id` (explicit ID) launch
   args still exist and override the preset's values individually if you
   need to point at an arbitrary file/marker without adding a preset.
4. `setup.py` globs `params/routes/*.yaml` into the install share dir —
   don't forget a clean rebuild (`util/build`) after adding a route file, or
   `bl.find` won't see it.

BT distances (`bt_dock_distance_m` etc.) are **not** currently bundled per
preset — they stay as top-level launch args shared by all routes. Revisit
this if a future station needs different docking distances than the
`parking_lot` one.

## Visualization (added this session, useful for building new routes)

`outdoor_stanley.py` now publishes three RViz/Foxglove `Marker` topics,
republished every ~1s:

- `outdoor_stanley/goal_marker` — blue sphere at the route's last waypoint.
- `outdoor_stanley/waypoints_marker` — yellow line through the raw
  `map_waypoints`.
- `outdoor_stanley/traj_marker` — green line showing the actual fitted spline
  (`controller.cx`/`cy`) — this is what the car is actually tracking, and
  where you'll see waviness if waypoints are under-constrained (see gotcha #1
  above).

Add all three in Foxglove/RViz when tuning a new route — the difference
between the yellow (raw waypoints) and green (fitted spline) lines is exactly
the diagnostic that caught gotcha #1 and #2 above.

## How to run / test

```bash
cd ~/Project/svea_charging
util/build
util/run
bl svea_charging outdoor_bt_charging.launch.py --enabled false
```

```bash
ros2 param get /self/outdoor_stanley enabled
ros2 topic echo /self/outdoor_stanley/status         # only publishes on change, not every tick
ros2 topic echo /self/mission/active_controller
ros2 topic echo /self/mission/phase
ros2 param set /self/outdoor_stanley enabled true
```

Record bags for debugging with:

```bash
cd /svea_ws/src
ros2 bag record -o <name> \
/self/odometry/global /self/odometry/gps \
/self/gps/carrier_solution /self/gps/horizontal_accuracy \
/self/mission/active_controller /self/mission/phase \
/self/outdoor_stanley/status /self/outdoor_stanley/course_heading \
/self/outdoor_stanley/cross_track_error /self/outdoor_stanley/yaw_error \
/self/stanley/cmd_velocity_mps /self/stanley/cmd_steering_rad \
/self/mavros/manual_control/send /self/mavros/wheel_odometry/velocity \
/rosout
```

Analysis requires ROS tooling that isn't available outside the dev
container — use `docker exec svea_charging ...` with
`source /svea_ws/install/setup.bash` and `rosbag2_py` to read `.mcap` bags if
your outer shell doesn't have ROS installed.

## Known-good state right now

Last full test run (test_24 in the session this handoff came from) completed
the entire route end to end with no corridor violations, no stalls, and
reasonable tracking — this is the first fully clean run of the whole
straight+turn route. The charging-station docking/charging phases past
`switch_distance_m` have **not** been end-to-end tested with real hardware.
That's where this branch should pick up.
