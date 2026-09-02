# Outdoor Localization for SVEA

## ROS 2 Jazzy checkpoint (2026-07-14)

This checkpoint records the first verified end-to-end outdoor localization
pipeline on SVEA3. It is intentionally kept here while alternative datum,
heading, and UTM configurations are evaluated.

### Verified data flow

```text
PX4 wheel velocity + PX4 IMU data_raw -> local EKF -> odometry/local
ZED-F9P NavSatFix + SWEPOS RTCM -> navsat_transform -> odometry/gps
wheel velocity + IMU yaw rate + odometry/gps -> global EKF -> odometry/global
local EKF:  self/odom -> self/base_link
global EKF: map -> self/odom
```

The stack is started through `svea_core/launch/svea.launch.py`, which includes
`svea_localization/launch/localization.launch.py` and the RTK/NTRIP launch.

The following outputs were verified:

- `/self/mavros/imu/data_raw`: approximately 45--47 Hz
- `/self/mavros/wheel_odometry/velocity`: approximately 10 Hz
- `/self/gps/fix`: approximately 1 Hz
- `/self/gps/rtcm`: approximately 5--6 Hz
- `/self/odometry/local`: approximately 20 Hz
- `/self/odometry/gps`: approximately 1 Hz
- `/self/odometry/global`: approximately 10 Hz
- TF chain: `map -> self/odom -> self/base_link`

### Checkpoint configuration

- Both EKFs fuse yaw rate from `mavros/imu/data_raw`.
- `data_raw` has no absolute orientation (`orientation_covariance[0] == -1`).
- Both EKFs are seeded with the launch argument `initial_pose_a` in ENU
  radians (`0=east`, `pi/2=north`).
- `navsat_transform_node` uses `use_odometry_yaw=true` and
  `yaw_offset=0.0`.
- `wait_for_datum=false`; the transform is initialized from live GPS and
  odometry data.
- The Cartesian/UTM transform is broadcast with the Cartesian frame as the
  parent.
- GPS odometry is fused as absolute X/Y data with
  `odom0_differential=false`.

### Known limitations

The current heading is initialized manually and then propagated using
integrated angular velocity. It is not continuously earth-referenced and can
drift. The upstream `robot_localization` documentation warns that
`use_odometry_yaw` should only be used when odometry yaw is earth-referenced.
This checkpoint is therefore a functional integration baseline, not yet a
validated production localization solution.

Occasional serial checksum errors and a startup warning about the GPS antenna
offset have also been observed. Data publication continued after both warnings.

### Smoke test

```bash
timeout 10s ros2 topic hz /self/odometry/gps
timeout 6s ros2 topic echo /self/odometry/gps --once
timeout 6s ros2 run tf2_ros tf2_echo map self/odom
timeout 6s ros2 run tf2_ros tf2_echo self/odom self/base_link
```

## ROS 2 validation logbook

### Working method

1. Keep the checkpoint configuration unchanged while a test is running.
2. Change one assumption or parameter at a time.
3. Record the launch command, physical setup, terminal output, and rosbag path.
4. Mark a test complete only after its result has been reviewed.
5. Keep failed tests in this log; they are part of the localization history.

### Current status

| Area | Status | Evidence or remaining question |
| --- | --- | --- |
| GNSS serial link | Verified | UBX NAV-PVT and two-way MON-VER communication |
| NTRIP/RTCM flow | Verified | SWEPOS connects and `/self/gps/rtcm` publishes |
| PX4/MAVROS link | Verified after PX4 reset | MAVROS connected; IMU raw and wheel velocity publish |
| Local EKF | Functionally verified | `/self/odometry/local` and `self/odom -> self/base_link` publish |
| GPS conversion | Functionally verified | `/self/odometry/gps` publishes at approximately 1 Hz |
| Global EKF | Functionally verified | `/self/odometry/global` and `map -> self/odom` publish |
| Stationary stability | In progress | Position spread and yaw drift have not been measured |
| Absolute heading | Limited | Manually initialized yaw followed by integrated yaw rate |
| RTK carrier solution | Not verified | RTCM flow does not prove RTK float or fixed |
| Startup ordering | Not verified | A transient GPS antenna-offset TF warning was observed |
| Clean shutdown/restart | Not verified | RTK manager previously kept the serial port locked |
| Fixed datum and UTM workflow | Deferred | Do not change until the baseline tests pass |

### Validation sequence

#### Step 1: stationary baseline — in progress

Purpose:

- Verify that every required topic remains active for at least 60 seconds.
- Measure stationary GPS position spread.
- Measure local and global odometry drift.
- Measure yaw drift caused by integrating `data_raw`.
- Save a repeatable baseline before changing the datum or filter settings.

Physical setup:

- Place the vehicle outdoors with open sky and do not move it during the test.
- Keep the remote controller available and the vehicle disarmed.
- Point the vehicle east and use `initial_pose_a=0.0`, or enter a measured ENU
  heading. Do not silently use zero for another direction.
- Record whether the PX4 had to be reset before launch.

Launch in terminal A:

```bash
bl svea_core svea.launch.py --initial_pose_a 0.0
```

Wait until MAVROS is connected, NTRIP is connected, and
`/self/odometry/gps` has started. In terminal B, run:

```bash
timeout 6s ros2 topic echo /self/mavros/state --once
timeout 10s ros2 topic hz /self/mavros/imu/data_raw
timeout 10s ros2 topic hz /self/mavros/wheel_odometry/velocity
timeout 10s ros2 topic hz /self/gps/fix
timeout 10s ros2 topic hz /self/odometry/local
timeout 10s ros2 topic hz /self/odometry/gps
timeout 10s ros2 topic hz /self/odometry/global
```

Record 60 seconds of stationary data:

```bash
BAG=/tmp/localization_stationary_$(date +%Y%m%d_%H%M%S)

timeout --signal=INT --kill-after=5s 60s ros2 bag record \
  -o "$BAG" \
  /self/mavros/state \
  /self/mavros/imu/data_raw \
  /self/mavros/wheel_odometry/velocity \
  /self/gps/fix \
  /self/gps/rtcm \
  /self/odometry/local \
  /self/odometry/gps \
  /self/odometry/global \
  /diagnostics \
  /tf \
  /tf_static

ros2 bag info "$BAG"
echo "$BAG"
```

Result template:

```text
Date/time:
Location and sky conditions:
Vehicle heading and initial_pose_a:
PX4 reset required: yes/no
MAVROS connected: yes/no
Bag path:
Observed topic rates:
Launch warnings/errors:
GPS X/Y spread:
Local odometry drift:
Global odometry drift:
Yaw change over 60 s:
Outcome: pass/fail/inconclusive
Notes:
```

The first run establishes measured baseline values. Numerical acceptance limits
will be selected after reviewing that data rather than chosen without evidence.

##### Step 1 run log

**2026-07-14, pre-recording topic check — pass**

The initial `ros2 topic hz` discovery warnings were followed by valid messages
and are therefore treated as startup/discovery warnings, not topic failures.

| Topic | Observed result |
| --- | --- |
| `/self/mavros/state` | Recheck passed: connected, disarmed, MANUAL |
| `/self/mavros/imu/data_raw` | Stabilized at approximately 48.4 Hz |
| `/self/mavros/wheel_odometry/velocity` | Approximately 10.0 Hz |
| `/self/gps/fix` | Approximately 1.0 Hz |
| `/self/odometry/local` | Approximately 19.9 Hz |
| `/self/odometry/gps` | Approximately 1.0 Hz |
| `/self/odometry/global` | Approximately 10.0 Hz |

The first state request occurred before topic discovery had completed. A later
request returned `connected: true`, `armed: false`, `manual_input: true`, and
`mode: MANUAL`. Interpretation: the complete measurement and filter data path
was active and the pre-recording check passed. The 60-second stationary rosbag
is the next active test.

#### Step 2: confirm the RTK carrier solution — pending

Confirm whether the ZED-F9P reports no carrier solution, RTK float, or RTK
fixed. The current `NavSatFix.status` mapping is insufficient, so this test
requires either exposing `carrSoln` as a ROS diagnostic or inspecting NAV-PVT
before the localization stack takes ownership of the serial port.

#### Step 3: controlled motion test — pending

Drive a measured straight segment, stop, turn, and return. Compare physical
distance and heading changes with local odometry, GPS odometry, and global
odometry. Perform this only after the stationary baseline has been reviewed.

#### Step 4: startup and TF ordering — pending

Repeat clean launches and check that the static sensor transforms and both EKFs
exist before `navsat_transform_node` initializes. The test passes when the GPS
antenna offset is applied without the transient `map -> base_link` error.

#### Step 5: clean shutdown and restart — pending

Stop the launch normally, confirm that no RTK, NTRIP, or MAVROS process remains,
and immediately launch again. The test passes when both serial devices can be
opened without force-killing old processes.

#### Step 6: fixed datum and explicit UTM workflow — deferred

Compare automatic startup datum with a fixed, repeatable map origin only after
steps 1--5 have established a reliable baseline. Preserve the automatic-datum
checkpoint as a selectable fallback.

The remainder of this document describes the older ROS 1 outdoor localization
stack and should not be treated as the current Jazzy launch procedure.

## Content
1. [Overview](#overview)
2. [Usage](#usage)
3. [Launch files](#launch-files)
4. [Config files](#config-files)
5. [Nodes](#nodes)

## Overview

Description for all the scripts that are related to the outdoor localization stack.

## Usage
1. Go to [this document](https://kth.sharepoint.com/:w:/s/ITRL/EQpnEBUVJVdMrDuXIj8IMBUBuqc_rFoeRelxt1d4YaZ71Q?e=Q4i3nz) (only for KTH team members) and copy one of the usersname and password to the rtk.launch file
2. Make sure the RTK-GPS is connected, and the realsense camera and 4K Logitech camera with the usb-c connection are disconnected. Ensure the SVEA is connected to the remote controller, so that you can always stop it when needed.
3. Start the outdoor localization stack with  
```
roslaunch svea_examples outdoor_test.launch device:=<port_location_for_rtk_gps>
```

## Launch files
### rtk.launch
This launch file is for starting the RTK-GPS unit.

Run the following to startup:
```
roslaunch svea_sensors rtk.launch device:=<port_location>
```
-   The port is fixed to `/dev/GPS` for SVEA2.

If the RTK-GPS stratup successfully, the terminal should show 
```
Connected to http://nrtk-swepos.lm.se:80/MSM_GNSS
```
To check the covariance and the GPS reading, use 
```
rostopic echo /gps/fix
```
The GPS reading is accurate enough when the covariance has a magnitude of less than 1e-4.

-   Notes:
    -   The GPS unit does not work well with the 4K Logitech camera and the realsense camera due to interference.
    -   The GPS must be placed far from the building, trees or other huge obstacles in order to obtain a more accurate reading.
    -   The username and password can be found in [this document](https://kth.sharepoint.com/:w:/s/ITRL/EQpnEBUVJVdMrDuXIj8IMBUBuqc_rFoeRelxt1d4YaZ71Q?e=Q4i3nz) (only for KTH team members).

### navsat.launch
This launch file is fro starting the navsat_transform_node, which is used to transform the RTK-GPS location to usable data for EKF global.

-   Notes:
    -   **`yaw_offset`**: The IMU has a default mapping for cardinal directions, i.e. for our IMU, 0 -> North. The assumption used for the navsat_transform_node is 0 -> East. Thus, this paramter should be set to pi/2. Unless the IMU is changed, you don't need to adjust this parameter.

    -   **`broadcast_utm_transform_as_parent_frame`**: To add the UTM frame as a parent of map frame.
    -    **`broadcast_utm_transform`**: To add the UTM frame. This parameter has to be set to true in order to use `broadcast_utm_transform_as_parent_frame`.

### rs_odometry.launch
This launch file is for starting the IMU, statc transforms, actuation_to_twist, EKF local and global. 

### transforms.launch
This lauch file is for starting all the required static transforms for the SVEA. 

### localize.launch
This launch file is for starting the rs_odometry.launch, serial_node, wheel_encoder, map, rtk.launch, navsat.launch and odom_to_map. 

General parameters

-   **`use_wheel_encoders`**: Set to be true if you are using svea7. Default: ``. 
-   **`start_serial`**: Must be set to true for motor. Default: `false`.
-   **`is_indoors`**: Must be set to false. Default: `true`.
-   **`device`**: The port where RTK-GPS is. Default:`/dev/GPS`.

### outdoor_test.launch
Autonomous driving with outdoor localization stack.
-   Parameters
    -   **`resolution`**: The number of division for each path. Default: `10`.
    -   **`corners`**: This parameter contains all the GPS coordinates (List of list that contains the lat, long of each point) that are the target points for the SVEA. To collect the GPS location for the target points, simply start the rtk.launch file, and drive the SVEA to the target positin in the physical environment. Make sure when you collect the GPS location, the rtk has a low covariance (magnitude of 1e-4 or smaller).
    -   **`use_wheel_encoders`**: true if the script is running on SVEA7.
    -   **`initial_pose_x`**, **`initial_pose_y`**, **`initial_pose_a`**: initial pose of SVEA (x,y,theta)
    -   *Below are for your reference, in most cases, you do not need to change these parameters*
        -   **`waypoints_topic`**: The topic which the target points to form the path are published to. Default: `/outdoor_localization_waypoint`.
        -   **`location_topic`**: The topic that publishes the GPS location of the SVEA. Default: `/gps/filtered`.
        -   **`marker_topic`**: The topic that publishes the visualization markers (The position of the target points and the initial pose of SVEA) in rViz. Default: `/waypoints`.
        -   **`gps_odometry_topic`**: The topic that publishes the SVEA pose (x,y) in map frame. Default: `/odometry/filtered/global`.
-   Note:
    -   **ALWAYS UNPLUG THE REALSENSE CAMERA AND THE 4K LOGITECH CAMERA (THE ONE WITH THE USBC CONNECTION) IF YOU ARE USING THE RTK-GPS**
    -   It might take a few trials before the RTK-GPS connects to the service.

## Config files

### global_ekf.yaml
Sensor fusion for EKF global. This includes `/odometry/gps`, `/imu/data`, `/actuation_twist`, `/wheel_encoder_twist`.
-   Parameters
    -   **`odom0_pose_rejection_threshold`**: To rejct inaccurate RTK-GPS measurement. If the RTK-GPS measurement is too far away (larger than the threshold) from the current location, the RTK-GPS data will be ignored. If wheel encoder is used, this value can be smaller.

### rs_ekf.yaml
Sensor fusion for EKF loval. This includes `/imu/data`, `/actuation_twist`, `/wheel_encoder_twist`.
-   Notes:
    -   **`/rs/t265_camera/odom/sample`**: This parameter is used for indoor localization, or when the RTK-GPS is not used.

## Nodes

### relative_waypoints.py
This node takes in the target points (in GPS coordinate) and calculates the relative distance between the target points and the initial position of the SVEA, and publishes this points in map frame (x,y) to `/outdoor_localization_waypoint`.

### outdoor_test.py
A node that has a very similar functionalities as the pure_pursuit.py, but it subscribes to `/outdoor_localization_waypoint` and use these points to generate the trajectory for pure pursuit.

### plot_localization.py
This node is for plotting the SVEA's path recorded by the RTK-GPS and EKF. It is best used for reviewing a rosbag. (Foxglove studio is also useful)
