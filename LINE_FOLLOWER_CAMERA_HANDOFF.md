# Line follower + camera tuning — handoff

## Context

Continuation of the work described in `CHARGING_STATION_HANDOFF.md` on branch
`bt_outdoor_charging_integration`. That file covers the BT/docking state
machine and the route-preset system (now committed, see below). This file is
a snapshot of a **live field-debugging session** that started right after
route presets landed — read `CHARGING_STATION_HANDOFF.md` first for the
overall picture, this is the more recent/narrower follow-up.

Two problems were found and diagnosed today, in this order:
1. Outdoor camera image was badly overexposed (blown-out white) — root
   caused, live-fixed via `v4l2-ctl`, **not yet baked into the launch file**.
2. With the camera fixed, `line_follower` still couldn't track the line
   outdoors — root caused via a captured rosbag, **fix values proposed but
   not yet validated on hardware or baked into code**.

Both fixes are "pick up right here" work — nothing below requires
re-deriving the diagnosis, just applying/testing the proposed values.

## Priorities for whoever picks this up

**#1 priority right now: get the camera image good and get `line_follower`
reliably tracking the line.** Everything else on this branch (new routes,
relocated station, `is_docked`/`battery_current` hardware verification) is
blocked on this, because none of it can be tested end-to-end without a
working visual docking approach. Don't get pulled into problem 3 (new route)
or the original BT/docking-hardware goal until line_follower is solid.

Keep the bigger picture in view while doing that, though: the actual goal of
this whole branch is getting the **BT integration** (Stanley →
`switch_distance_m` handoff → `line_follower` → dock → charge → resume) to
work end-to-end against a real charging station. The camera/line-follower
work is a blocker for that, not a separate side quest — don't over-invest in
line-following in isolation (e.g. tuning it to track perfectly on an empty
lot with no BT running) if it means neglecting the docking/charging
verification once the image is usable.

**Do not start by changing parameters.** Before touching `v4l2-ctl`,
`ros2 param set`, or any code defaults: first look at what's actually there.
Check the current camera image (is it still blown out / cyan-tinted / does
it look different from what's described below — camera hardware differs
per-robot, see the multi-robot note), check the current HSV mask against
`line_follower`'s current thresholds using the same rosbag/frame-extraction
technique documented below, and check whether the specific fixes proposed in
this file even still apply to whatever robot you're on. Only start changing
values once you've confirmed *what's actually wrong on this specific robot
right now* — don't assume yesterday's diagnosis transfers unchanged (it
didn't, see below).

## Multi-robot note (important — found on svea7)

The camera exposure/white-balance fix and the line-follower HSV thresholds
are **per-robot, not global**. Confirmed by switching from svea3 to svea7
mid-session: svea7's camera came up with a much stronger cyan/blue tint than
svea3 ever showed (even after svea3's `v4l2-ctl` fix), and `line_follower`
was back to `status=line_lost` immediately — because the `v4l2-ctl` fix from
problem 1 was only ever applied live to svea3's running `usb_cam_node`
process, never baked into the launch file (see "Not done yet" below), so a
fresh robot/process starts from the hardware's own defaults again. Each
physical camera unit likely also has its own auto-exposure/white-balance
quirks even at the same settings.

Practical implication: **redo the "look before touching parameters" check
per robot.** Don't assume svea3's tuned values (`exposure_time_absolute=150`,
`white_balance_temperature=4500`, HSV thresholds proposed under problem 2)
are correct on a different physical car — treat them as a starting point /
prior, not a given, and re-derive from that robot's own image data.

Also confirmed while debugging svea7: the hardcoded `rtk_device` default
(`/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR_WiFi_1010_...`, shared across
`svea_core/svea.launch.py`, `svea_localization/sensors/rtk.launch.py`, and
both `svea_charging` launch files) turned out to **not** be a per-robot
problem — the MKR WiFi 1010 boards apparently don't have unique USB serial
numbers, so the same by-id path resolved correctly (to a different `ttyACM*`
each time) on both svea3 and svea7. GPS/RTK itself was confirmed fully
healthy on svea7 (`carrier_solution=2` i.e. RTK Fixed, 31 satellites, good
covariance) — that was a red herring, not a real bug. Mentioned here only so
it isn't re-investigated from scratch if it comes up again.

## Repo state right now

- Route presets (`route_preset` launch arg, `params/routes/` directory) are
  **committed** at `9be27b4` ("added options to have multiple routes") — see
  `CHARGING_STATION_HANDOFF.md` for details, that part is done and verified
  working (confirmed via a real `util/run` launch log: `outdoor_stanley`
  picked up `params/routes/to_parking_lot.yaml` and `aruco_camera_test`
  picked up `marker_id:=11`, both resolved automatically from
  `route_preset=parking_lot`).
- **Uncommitted change**: `src/svea_charging/scripts/line_follower.py` —
  `publish_debug_image` default flipped from `False` to `True` (line ~78),
  done live in the field to help inspect the line-detection mask. Leave this
  on while debugging; consider whether it should default back to `False`
  once tuning is done (it publishes `line_follower/debug_image` every 3rd
  frame — cheap, probably fine to leave on, your call).
- A large number of untracked directories are field-recorded rosbags/test
  runs (`src/outdoor_bt_test_*/`, `src/rosbags/`, `src/record_waypoints/`,
  etc.) — not part of the intended diff, don't stage them into a commit
  without checking with the user first.
- Useful trick discovered this session: you can read `.mcap` rosbag2 bags
  **without any ROS install**, using the pure-Python `rosbags` package:
  ```bash
  pip install rosbags
  python3 -c "
  from rosbags.rosbag2 import Reader
  from pathlib import Path
  with Reader(Path('path/to/bag_dir')) as r:
      for c in r.connections: print(c.topic, c.msgtype, c.msgcount)
  "
  ```
  Use `rosbags.typesys.get_typestore(Stores.ROS2_JAZZY)` +
  `typestore.deserialize_cdr(raw, conn.msgtype)` to decode individual
  messages. This is much faster than spinning up the container just to
  inspect a bag, and was how both bugs below were confirmed from
  `src/rosbags/line_followertest_02.bag`.

## Problem 1: camera overexposed outdoors

### Root cause

`usb_cam_node`'s parameters (`autoexposure`, `exposure`, `auto_white_balance`,
`white_balance` — set in `outdoor_bt_charging.launch.py`) map to **v4l2
control names that don't exist on this camera's driver**. Confirmed from the
launch log:
```
unknown control 'white_balance_temperature_auto'
white_balance_temperature: Permission denied
VIDIOC_S_CTRL: failed: Permission denied
unknown control 'exposure_auto'
unknown control 'exposure_absolute'
unknown control 'focus_auto'
```
So `autoexposure:=false` silently never took effect — the camera stayed in
its default auto-exposure mode (`auto_exposure` menu control, UVC standard,
confusingly `1 = Manual Mode` / `3 = Aperture Priority Mode` which is what it
defaults to), which is why it was blowing out in direct sunlight regardless
of what `exposure` was set to.

Note: `brightness` and `gain` **do** work through the node's own params —
their control names happen to match — only the exposure/white-balance
auto-toggle controls are misnamed for this hardware.

### Real control names (from `v4l2-ctl -d /dev/video0 --list-ctrls`)

```
auto_exposure              menu, 0-3, default=3 (Aperture Priority = auto)
exposure_time_absolute     int,  min=3 max=2047, default=250 (inactive while auto_exposure≠1)
white_balance_automatic    bool, default=1
white_balance_temperature  int,  min=2000 max=7500 step=10, default=4000 (inactive while automatic=1)
brightness                 int,  min=0 max=255, default=128 (this one matches the node's own param)
gain                       int,  min=0 max=255, default=0   (this one also matches)
```

### Live fix applied in the field (works, bypasses the ROS node)

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1          # 1 = Manual Mode
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=150
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=4500
```
This got the image out of "blown white" territory (confirmed visually in
Foxglove — see the two screenshots discussed live, image went from
all-white/washed to a normally exposed but slightly cool/blue-tinted scene).
The blue tint is a strong hint `white_balance_temperature=4500` isn't quite
right for actual ambient daylight (probably needs to go higher, try
5500-6500, adjust while watching the image) — **this value was not
fine-tuned**, just gotten out of the broken state. It also may be
contributing to problem 2 below (desaturating the yellow line).

### Not done yet

- Find better `exposure_time_absolute` / `white_balance_temperature` values
  (current ones are "good enough to not be blown out", not tuned).
- **Bake the fix into the launch file permanently.** Since it must go
  through `v4l2-ctl` and not through `usb_cam_node`'s own (broken-for-this-
  camera) params, the cleanest way is probably an `ExecuteProcess`/shell step
  in `outdoor_bt_charging.launch.py` that runs the four `v4l2-ctl` commands
  right after `usb_cam_node` starts (or a tiny wrapper script called from
  the launch file). Not implemented — needs a decision on where that step
  belongs in `bl.node(...)`/`bl.group(...)` structure.

## Problem 2: line_follower can't see the line outdoors

### Root cause (confirmed from `src/rosbags/line_followertest_02.bag`)

`line_follower/status` was `"line_lost"` in **1481 of 1489** loop iterations
(99.5%) during that test — matches what was seen live in Foxglove
(`line_follower/error_px` never publishes at all, because the code only
publishes it on a successful detection; `cmd_velocity_mps`/`cmd_steering_rad`
were being driven to 0 the whole time because `stop_on_lost_line=True`).

Extracted a real frame from the bag and ran the exact same HSV mask the node
uses (`_extract_line_centroid`,
[line_follower.py:202-230](src/svea_charging/scripts/line_follower.py#L202-L230)).
Current thresholds (module-level params, lines ~82-87):
```python
lower_h = 20; lower_s = 100; lower_v = 100
upper_h = 35; upper_s = 255; upper_v = 255
```
Actual measured HSV of the yellow line's pixels in the outdoor frame:
```
H ≈ 19-23   (right at/under lower_h=20 — some pixels excluded)
S ≈ 50-90   (well under lower_s=100 — this is the actual failure)
V ≈ 176-189 (fine, within range)
```
Of 138,240 ROI pixels (bottom 45% of frame, `crop_start_ratio=0.55`), only
**8** matched the mask. The line outdoors is simply much less saturated than
whatever it was tuned against indoors (this may partly be the
`white_balance_temperature` value from problem 1 pulling saturation down —
worth re-checking this threshold after white balance is properly tuned, not
just after being "unbroken").

### Proposed fix (not yet validated on hardware)

```bash
ros2 param set /self/line_follower lower_h 15
ros2 param set /self/line_follower lower_s 40
ros2 param set /self/line_follower lower_v 80
```
Test live, watch `line_follower/status` (want mostly `tracking`, not
`line_lost`) and/or the debug image (`line_follower/debug_image`, now
publishing by default per the uncommitted change above). Once good values
are found, update the defaults in `line_follower.py` (lines 82-87) to make
them permanent.

### Suggested next step not yet done

A proper sweep of the *entire* bag's HSV distribution (not just one frame)
to pick thresholds that are robust across the whole test, rather than eyeballing
one frame — offered to the user but not run yet. Quick to do with the same
`rosbags` + `cv2` approach used for the single-frame check above: decode all
`/self/image_raw` frames, `cv2.cvtColor` to HSV, histogram the pixels that
are visually part of the line (e.g. via the same `yellowish-by-eye` BGR
heuristic used in the one-frame check), take robust min/max percentiles.

## Problem 3 (parked, not started): new route with earlier turn + relocated station

Separate thread, paused before the camera/line-follower detour. User wants:
- Same route as today's `to_parking_lot` preset, but the right turn happens
  **earlier** along the first straight leg.
- After that right turn, a **new** (not-too-long) segment including a left
  turn, ending at a **relocated** charging station.
- Station should be approached via an exact straight line for several meters
  before it, for a clean handover pose.
- Agreed approach: place the station physically first, then take stationary
  RTK anchor reads (same technique as `CHARGING_STATION_HANDOFF.md` route
  gotchas) at: the station itself, a lead-in point a few meters behind it,
  the new right-turn point, and the new left-turn point. Build the route
  from those anchors (straight legs at ~1m spacing, fillet arcs at ~0.5m
  spacing through each turn, curvature-checked), save as a new preset
  (e.g. `to_parking_lot_v2` or similar) alongside the existing one rather
  than overwriting it.
- **Blocked on:** none of these coordinates have been collected yet. This is
  the next physical-world step once camera/line-follower are sorted, since
  you need the line follower working to actually test the new station
  approach end-to-end.

## Suggested order to pick this up

0. **Look before touching anything.** On whatever robot you're on right now:
   pull a current `/self/image_raw` frame (live `ros2 topic echo --once` +
   `cv_bridge`, or grab one from a fresh short rosbag) and check its actual
   exposure/color by eye and via the same HSV-measurement approach used
   below — don't assume it's still blown out, or still cyan, or that
   svea3/svea7's numbers apply. Confirm `line_follower/status` is actually
   `line_lost` right now before assuming the same bug applies.
1. Only then start adjusting: get the image reasonably exposed/color-correct
   via `v4l2-ctl` (see problem 1's real control names below — they may or
   may not be the same on this robot's camera, check
   `v4l2-ctl -d /dev/video0 --list-ctrls` fresh rather than assuming).
2. Validate/derive line_follower HSV thresholds against that specific
   image (same technique as problem 2 below).
3. Bake both the v4l2-ctl camera fix and the HSV thresholds into
   `outdoor_bt_charging.launch.py` / `line_follower.py` so they survive a
   fresh `util/run` without manual `ros2 param set`/`v4l2-ctl` steps — and so
   the next robot/session doesn't repeat this whole loop from scratch.
4. Once line_follower reliably tracks outdoors, this branch's actual goal
   picks back up: verify the BT hands off cleanly end-to-end and dock/charge
   against a real charging station (`is_docked`/`battery_current`/`aux4` from
   `CHARGING_STATION_HANDOFF.md` — still unverified on real hardware).
5. Only after that: resume problem 3 (new route with earlier turn +
   relocated station) — collect the anchor coordinates.
