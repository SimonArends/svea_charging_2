# svea_charging

`svea_charging` contains the current SVEA charging behavior and the indoor
charging demo setup. The package is being structured so the charging behavior
can be reused from another ROS 2 workspace without forcing that workspace to use
the current indoor MoCap setup.

## What This Package Owns

This package owns the charging-specific parts of the stack:

- the behavior-tree runner that selects the active charging phase
- the Stanley approach controller
- the line-following final docking controller
- the controller mux that forwards the active controller command to SVEA
- the ArUco marker detector used for short-range charger detection
- launch files for the reusable charging behavior and the current indoor demo

Robot bringup, map server, visualization, and the localization source are kept
outside the reusable launch entry point. For the current indoor demo they are
started by `indoor_charging_demo.xml`.

## Launch Files

### `bt_charging.xml`

This is the reusable charging launch. It starts only the charging nodes:

- `aruco_camera_test.py`
- `stanleyExecutable.py`
- `line_follower.py`
- `bt_runner.py`
- `control_mux.py`

It expects another launch file to provide:

- SVEA robot bringup
- localization
- camera image input
- map and visualization, if wanted

Example include:

```xml
<include file="$(find-pkg-share svea_charging)/launch/bt_charging.xml">
  <arg name="name" value="svea67"/>
  <arg name="is_sim" value="false"/>
  <arg name="use_mocap" value="true"/>
  <arg name="base_frame" value="svea67/base_link"/>
  <arg name="camera_image_topic" value="image_raw"/>
</include>
```

Important arguments:

- `name`: ROS namespace for the robot and charging nodes.
- `is_sim`: passed to the controllers and mux.
- `use_mocap`: makes the Stanley controller read the current indoor MoCap pose
  topics instead of the default `LocalizationInterface` state.
- `base_frame`: base frame used by the localization interface.
- `camera_image_topic`: image topic consumed by ArUco and line following.
- `use_aruco_camera`: starts or disables the ArUco detector.
- `bt_switch_distance_m`: distance at which the behavior tree switches from
  Stanley approach to line following.
- `bt_dock_distance_m`: distance threshold used for the docking/charging phase.
- `aruco_calibration_file`: camera calibration file used for marker distance.

### `indoor_charging_demo.xml`

This is the current full indoor demo launch. It keeps the working setup together:

- starts the SML map and Foxglove through `svea_core`
- starts simulated SVEA through `svea.xml` when `is_sim=true`
- starts real SVEA with MoCap through `svea_mocap.xml` when `is_sim=false`
- starts `usb_cam` for the real indoor vehicle
- includes `bt_charging.xml` for the charging behavior

Run the current real indoor demo with:

```bash
ros2 launch svea_charging indoor_charging_demo.xml
```

Run the simulation path with:

```bash
ros2 launch svea_charging indoor_charging_demo.xml is_sim:=true use_mocap:=false
```

## Indoor Charging Solution

The indoor solution is a staged docking behavior:

1. `bt_runner.py` ticks the charging behavior tree and publishes the active
   controller on `mission/active_controller`.
2. `stanleyExecutable.py` performs the first approach toward the charging
   station. In the current real indoor setup, `use_mocap=true` makes it read:
   - `/mocap/svea/pose`
   - `/mocap/charging_station/pose`
3. `aruco_camera_test.py` detects the charging station marker from the camera
   image and publishes:
   - `aruco/poses`
   - `aruco/distance_m`
   - `aruco/status`
   - `aruco/debug_image/compressed`
4. `line_follower.py` follows the visual line for final alignment and uses
   `aruco/distance_m` to stop near the charger.
5. `control_mux.py` listens to the active controller selected by the behavior
   tree and sends only that controller's steering/velocity commands to SVEA.

The behavior tree switches between controllers using distance and perception
signals:

- Stanley approach publishes `dist_to_goal`.
- ArUco publishes `aruco/distance_m`.
- Line following publishes `line_follower/status`.
- The behavior tree publishes `mission/phase`, `mission/tree_status`, and
  `mission/active_controller`.

## Indoor Assumptions

The current indoor demo assumes:

- robot namespace: `svea67`
- real robot base frame: `svea67/base_link`
- map: `sml`
- camera device: `/dev/video0`
- image topic inside the robot namespace: `image_raw`
- ArUco dictionary: `DICT_4X4_50`
- ArUco marker ID: `13`
- ArUco marker length: `0.365` m
- camera calibration: `params/camera.yaml`
- MoCap pose topics:
  - `/mocap/svea/pose`
  - `/mocap/charging_station/pose`

Most of these are launch arguments in `indoor_charging_demo.xml`, so another
robot or lab setup should be able to override them without editing the scripts.

## Topics

Core inputs:

- `image_raw`: camera image used by ArUco and line following.
- `/mocap/svea/pose`: indoor vehicle pose when `use_mocap=true`.
- `/mocap/charging_station/pose`: indoor charging-station pose when the Stanley
  controller uses a MoCap goal.
- `/lli/battery/state`: battery state used by the behavior tree.

Core outputs:

- `mission/active_controller`: selected controller, for example `stanley`,
  `line_follower`, or `idle`.
- `mission/phase`: current behavior-tree phase.
- `mission/tree_status`: behavior-tree tick status.
- `stanley/cmd_steering_rad` and `stanley/cmd_velocity_mps`.
- `line_follower/cmd_steering_rad` and `line_follower/cmd_velocity_mps`.
- `aruco/distance_m`, `aruco/poses`, and `aruco/status`.
- `/charging_arm`: charging-arm command.

## Path Toward Outdoor GPS/RTK

The reusable package boundary is now:

- `bt_charging.xml` starts charging behavior only.
- localization is an external provider.
- the including launch file decides whether pose comes from MoCap, simulation,
  GPS/RTK, EKF, or another source.

The next outdoor step should be to make the approach controller consume a
localization interface and station pose config instead of indoor-only MoCap
topics. After that, a GPS/RTK launch can provide the same robot pose and station
reference without changing the behavior-tree interface.
