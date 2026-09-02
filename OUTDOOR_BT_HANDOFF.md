# Outdoor BT Charging handoff

Goal: continue the outdoor charging demo work without re-explaining context.

## Current objective

Build a first outdoor demo:

1. RTK/GPS + encoders drive a coarse approach path with Stanley.
2. Behaviour tree keeps same charging flow as `bt_charging.launch.py`.
3. BT switches from Stanley to `line_follower` only when ArUco says the charging station is close/visible.
4. Line follower handles final docking.

No GPS fallback handover for now. We explicitly decided not to switch to line follower based only on GPS goal distance, because if ArUco/line is not visible then line follower is blind too.

## Important architecture decision

`control_mux.py` must be the only thing that sends actuation to the car in BT mode.

So `outdoor_stanley.py` now supports:

- `command_mode="direct"`: old standalone mode, sends directly via `ActuationInterface`.
- `command_mode="topic"`: BT mode, publishes:
  - `stanley/cmd_steering_rad`
  - `stanley/cmd_velocity_mps`

In topic mode it only runs when:

```text
/self/mission/active_controller == "stanley"
```

This preserves the indoor/mocap BT/mux/line_follower structure.

## Main files changed/added

- `src/svea_charging/scripts/outdoor_stanley.py`
  - supports `command_mode`
  - listens to `mission/active_controller`
  - publishes `stanley/cmd_*`
  - publishes debug/status topics
  - clamps velocity and steering

- `src/svea_charging/launch/outdoor_bt_charging.launch.py`
  - new launch file for outdoor BT charging
  - starts RTK/GPS localization, camera/ArUco, outdoor Stanley, line follower, BT runner, control mux

- `src/svea_charging/params/outdoor_route.yaml`
  - route updated with Test 03 straight-line endpoint:
    - `[24.88, -9.81]`
  - this point is intended as end of straight and start of right turn
  - tuning changed:
    - `course_heading_min_distance: 0.50`
    - `course_heading_alpha: 0.20`

## Current launch defaults

In `outdoor_bt_charging.launch.py`:

```python
stanley_target_velocity = 0.80
stanley_turn_velocity = 0.30
stanley_max_steering_rad = 0.35
control_mux_timeout_s = 1.0
```

Reason:

- Test 04/05 showed Stanley/BT worked but car did not move.
- `manual_control/send.z` reached only ~612 in Test 04 and ~662 in Test 05.
- wheel odometry stayed exactly zero.
- Increased target velocity to get a stronger throttle command.
- Increased control mux timeout because `manual_control/send.z` was toggling between neutral and throttle.

## How to run

Build:

```bash
cd ~/Project/svea_charging
util/build
```

Start:

```bash
util/run
bl svea_charging outdoor_bt_charging.launch.py --enabled false
```

Check:

```bash
ros2 topic echo --once /self/gps/carrier_solution
ros2 topic echo --once /self/gps/horizontal_accuracy
ros2 topic echo --once /self/odometry/global
ros2 topic echo --once /self/mission/active_controller
ros2 param get /self/outdoor_stanley enabled
```

Activate:

```bash
ros2 param set /self/outdoor_stanley enabled true
```

Stop:

```bash
ros2 param set /self/outdoor_stanley enabled false
```

## Record bag

Run from `/svea_ws/src`:

```bash
ros2 bag record -o outdoor_bt_test_XX \
/self/odometry/global /self/odometry/gps \
/self/gps/carrier_solution /self/gps/horizontal_accuracy \
/self/mission/active_controller /self/mission/phase \
/self/outdoor_stanley/status /self/outdoor_stanley/course_heading \
/self/outdoor_stanley/cross_track_error /self/outdoor_stanley/yaw_error \
/self/stanley/cmd_velocity_mps /self/stanley/cmd_steering_rad \
/self/mavros/manual_control/send /self/mavros/wheel_odometry/velocity \
/rosout
```

## Test results so far

### Test 03

Bag: `src/outdoor_bt_test_03`

Important result:

- RTK perfect:
  - `carrier_solution=2` throughout
  - hAcc about `0.014 m`
- BT correct:
  - `active_controller=stanley`
  - phase `approach`
- Stanley status:
  - `running`
- Velocity clamp worked:
  - `cmd_velocity_mps = 0.38`
- Vehicle moved:
  - global start approx `(0.086, 0.100)`
  - global end approx `(24.88, -9.81)`
  - net distance approx `26.7 m`
- User said this end point is where the straight line should end and right turn should begin.
- Cross-track was acceptable:
  - min approx `-0.67 m`
  - max approx `0.45 m`
  - mean approx `-0.01 m`
- Oscillation existed:
  - steering hit clamp often
  - steering sign changes about `52`

Conclusion:

- Straight line is good enough but oscillatory.
- We tuned heading smoothing and steering clamp afterward.

### Test 04

Bag: `src/outdoor_bt_test_04`

After tuning:

- `cmd_velocity_mps = 0.38`
- `cmd_steering_rad` small, roughly `-0.03` to `-0.14`
- `manual_control/send.z` toggled between `500` and `611.8`
- wheel odometry:
  - `vx = 0.0` throughout

Conclusion:

- Software was commanding throttle.
- Car did not move.
- Not a GPS/Stanley path issue.

### Test 05

Bag: `src/outdoor_bt_test_05`

After raising target velocity to `0.55`:

- `cmd_velocity_mps = 0.55`
- `manual_control/send.z` toggled between `500` and `661.8`
- wheel odometry:
  - `vx = 0.0` throughout

Conclusion:

- Software still commands throttle.
- Car still does not move.
- Likely low-level actuation / drive-mode / throttle threshold / mux timing issue.

## Current suspected issue

The car is not moving even though:

- BT selects Stanley
- outdoor Stanley is `running`
- Stanley publishes `cmd_velocity_mps`
- control mux publishes non-neutral `/self/mavros/manual_control/send.z`
- RTK and odometry are alive

But:

```text
/self/mavros/wheel_odometry/velocity.twist.twist.linear.x == 0.0
```

So the next debugging should focus on low-level drive output:

1. Is the car armed / in correct mode / accepting manual_control?
2. Does `manual_control/send.z ≈ 662` or higher normally move the car?
3. Is the velocity sign correct through `control_mux`?
   - `control_mux.py` sends:
     - `self.actuation.send_control(cmd.steering, -1*cmd.velocity)`
4. Is `controller_timeout_s` causing neutral pulses?
   - Now patched to `1.0`.
5. Does manual driving through the same interface work while this launch is running?

## Very next test

After rebuilding and starting the updated launch:

1. Verify command:

```bash
ros2 topic echo --once /self/stanley/cmd_velocity_mps
ros2 topic echo /self/mavros/manual_control/send
```

Expected:

- `cmd_velocity_mps` near `0.8`
- `manual_control/send.z` higher than previous tests and less pulsed

2. If wheel odom still stays zero:

```bash
ros2 topic echo /self/mavros/wheel_odometry/velocity
```

If still zero, stop chasing Stanley and debug LLI/manual-control/arming.

## Notes

- Outdoor EKF currently uses GPS/encoders, no IMU yaw for outdoor.
- Stanley yaw is course-heading from RTK position deltas.
- This is OK for coarse forward demo, not precise parking.
- Start pose still matters: place car near first waypoint and aligned with first segment.
- Current route still ends at charging station area; BT switch is based only on ArUco-distance.

cd /svea_ws/src
ros2 bag record -o line_followertest_02.bag \
/self/odometry/global /self/odometry/gps \
/self/gps/carrier_solution /self/gps/horizontal_accuracy \
/self/mission/active_controller /self/mission/phase \
/self/outdoor_stanley/status /self/outdoor_stanley/course_heading \
/self/outdoor_stanley/cross_track_error /self/outdoor_stanley/yaw_error \
/self/stanley/cmd_velocity_mps /self/stanley/cmd_steering_rad \
/self/mavros/manual_control/send /self/mavros/wheel_odometry/velocity \
/rosout

ros2 bag record -a -o line_followertest_02.bag





ros2 run usb_cam usb_cam_node_exe \
  --ros-args \
  -r /image_raw:=image_raw \
  -r /camera_info:=camera/camera_info \
  -p video_device:=/dev/video0 \
  -p camera_name:=narrow_stereo \
  -p frame_id:=self/camera \
  -p pixel_format:=mjpeg2rgb \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p camera_info_url:=file:///root/svea3/svea_ws/src/svea_charging/params/camera.yaml \
  -p brightness:=120 \
  -p gain:=10 \
  -p auto_white_balance:=false \
  -p white_balance:=4000 \
  -p autoexposure:=false \
  -p exposure:=700 \
  -p autofocus:=true \
  -p focus:=-1


ros2 bag record -o post_turn_check_03 /self/odometry/global /self/odometry/gps /self/gps/carrier_solution /self/gps/horizontal_accuracy /self/outdoor_stanley/status /self/outdoor_stanley/course_heading /self/outdoor_stanley/cross_track_error /self/outdoor_stanley/yaw_error /self/outdoor_stanley/target_index /self/stanley/cmd_velocity_mps /self/stanley/cmd_steering_rad /self/mission/active_controller /self/mission/phase /self/aruco/distance_m /self/mavros/manual_control/send /self/mavros/wheel_odometry/velocity /rosout