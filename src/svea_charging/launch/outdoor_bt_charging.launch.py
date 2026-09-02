#!/usr/bin/env python3

from better_launch import BetterLaunch, launch_this

# Each preset bundles a route file with the ArUco marker ID mounted on that
# route's charging station, so the two can never drift out of sync (see
# CHARGING_STATION_HANDOFF.md - route + marker ID are logically one
# "destination"). Add a new preset here whenever a new route/station is
# recorded and calibrated.
ROUTE_PRESETS = {
    "parking_lot": dict(
        route_config="params/routes/to_parking_lot.yaml",
        aruco_marker_id=11,
    ),
    # Earlier right turn + relocated charging station (same physical
    # station/marker, just moved) — see to_charging_station.yaml for the
    # anchor-point derivation. Not yet driven end-to-end; verify the
    # curvature-checked geometry against the real turn before trusting it
    # at full speed. Update aruco_marker_id here if a new/different marker
    # was printed for the relocated station.
    "charging_station": dict(
        route_config="params/routes/to_charging_station.yaml",
        aruco_marker_id=11,
    ),
    # Second, alternate charging station location - same right-turn-then-
    # straight approach, different physical station and anchor points, see
    # to_charging_station_2.yaml for derivation. Not yet driven. Update
    # aruco_marker_id here if this station uses a different physical
    # marker than the others.
    "charging_station_2": dict(
        route_config="params/routes/to_charging_station_2.yaml",
        aruco_marker_id=11,
    ),
}


@launch_this
def main(
    name: str = "self",
    enabled: bool = True,
    initial_pose_a: float = 2.548181,
    route_preset: str = "charging_station",
    route_config: str = "",
    use_datum: bool = True,
    datum_file: str = "",
    rtk_device: str = "/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR_WiFi_1010_C5EE644B5150484347202020FF0E0B39-if00",
    rtk_baud: int = 115200,
    rtk_username: str = "ITRL03",
    rtk_password: str = "171488",
    use_foxglove: bool = True,
    use_aruco_camera: bool = True,
    camera_image_topic: str = "image_raw",
    camera_frame_id: str = "{name}/camera",
    aruco_dictionary: str = "DICT_4X4_50",
    aruco_marker_length_m: float = 0.075,
    aruco_display: bool = False,
    aruco_loop_hz: float = 30.0,
    aruco_frame_id: str = "{name}/camera",
    aruco_use_aruco_detector_api: bool = False,
    aruco_publish_debug_image: bool = False,
    aruco_jpeg_quality: int = 80,
    aruco_generate_marker_on_startup: bool = False,
    aruco_marker_id: int = -1,
    aruco_marker_size_px: int = 400,
    aruco_output: str = "aruco_marker.png",
    aruco_calibration_file: str = "",
    aruco_focal_length_px: float = -1.0,
    bt_dock_distance_m: float = 0.6251276731491089,
    bt_switch_distance_m: float = 2.25,
    bt_docking_exit_distance_m: float = 2.75,
    bt_charge_start_voltage: float = 12.6,
    bt_charge_done_voltage: float = 12.6,
    bt_charge_voltage_confirm_s: float = 3.0,
    stanley_target_velocity: float = 0.28,
    stanley_turn_velocity: float = 0.3,
    stanley_max_steering_rad: float = 0.35,
    control_mux_timeout_s: float = 1.0,
    camera_video_device: str = "/dev/video0",
    apply_camera_v4l2_fix: bool = False,
    camera_exposure_time_absolute: int = 150,
    camera_white_balance_temperature: int = 6500,
):
    """Outdoor charging mission: RTK/GPS Stanley approach, then line follower."""
    bl = BetterLaunch()

    camera_frame_id = camera_frame_id.format(name=name)
    aruco_frame_id = aruco_frame_id.format(name=name)

    if route_preset:
        if route_preset not in ROUTE_PRESETS:
            raise ValueError(
                f"Unknown route_preset '{route_preset}', expected one of "
                f"{list(ROUTE_PRESETS)}"
            )
        preset = ROUTE_PRESETS[route_preset]
        if not route_config:
            route_config = bl.find("svea_charging", preset["route_config"])
        if aruco_marker_id < 0:
            aruco_marker_id = preset["aruco_marker_id"]

    if not route_config:
        route_config = bl.find("svea_charging", "params/routes/to_parking_lot.yaml")
    if aruco_marker_id < 0:
        aruco_marker_id = 11
    if not datum_file:
        datum_file = bl.find("svea_charging", "params/outdoor_datum.yaml")
    if not aruco_calibration_file:
        aruco_calibration_file = bl.find("svea_charging", "params/camera.yaml")

    bl.include(
        "svea_core",
        "svea.launch.py",
        name=name,
        is_sim=False,
        is_indoor=False,
        initial_pose_a=initial_pose_a,
        use_localization=True,
        use_map=False,
        use_rtk=True,
        rtk_device=rtk_device,
        rtk_baud=rtk_baud,
        rtk_username=rtk_username,
        rtk_password=rtk_password,
        use_datum=use_datum,
        datum_service="datum",
        datum_file=datum_file,
        use_foxglove=use_foxglove,
    )

    with bl.group(name):
        bl.node(
            "usb_cam",
            "usb_cam_node_exe",
            name="usb_cam_node",
            params=dict(
                video_device=camera_video_device,
                camera_name="narrow_stereo",
                frame_id=camera_frame_id,
                pixel_format="mjpeg2rgb",
                image_width=640,
                image_height=480,
                framerate=30.0,
                camera_info_url=f"file://{aruco_calibration_file}",
                brightness=120,
                gain=10,
                auto_white_balance=False,
                white_balance=4000,
                autoexposure=False,
                exposure=700,
                autofocus=True,
                focus=-1,
            ),
            remaps={
                "/image_raw": camera_image_topic,
                "/camera_info": "camera/camera_info",
            },
        )

        if apply_camera_v4l2_fix:
            # usb_cam_node's own autoexposure/auto_white_balance params
            # silently no-op on this camera's v4l2 driver (control names
            # don't match), so the fix has to go through v4l2-ctl directly.
            # See LINE_FOLLOWER_CAMERA_HANDOFF.md problem 1. Values are
            # per-robot/per-camera - override via launch args if they don't
            # hold on a different unit.
            bl.node(
                "svea_charging",
                "set_camera_ctrls.py",
                name="set_camera_ctrls",
                params=dict(
                    video_device=camera_video_device,
                    exposure_time_absolute=camera_exposure_time_absolute,
                    white_balance_temperature=camera_white_balance_temperature,
                ),
            )

        if use_aruco_camera:
            bl.node(
                "svea_charging",
                "aruco_camera_test.py",
                name="aruco_camera_test",
                params=dict(
                    dictionary=aruco_dictionary,
                    marker_length_m=aruco_marker_length_m,
                    display=aruco_display,
                    loop_hz=aruco_loop_hz,
                    frame_id=aruco_frame_id,
                    use_aruco_detector_api=aruco_use_aruco_detector_api,
                    publish_debug_image=aruco_publish_debug_image,
                    jpeg_quality=aruco_jpeg_quality,
                    generate_marker_on_startup=aruco_generate_marker_on_startup,
                    marker_id=aruco_marker_id,
                    marker_size_px=aruco_marker_size_px,
                    output=aruco_output,
                    calibration_file=aruco_calibration_file,
                    focal_length_px=aruco_focal_length_px,
                    image_topic=camera_image_topic,
                ),
            )

        bl.node(
            "svea_charging",
            "outdoor_stanley.py",
            name="outdoor_stanley",
            param_files=route_config,
            params=dict(
                enabled=enabled,
                controller_name="stanley",
                target_velocity=stanley_target_velocity,
                turn_velocity=stanley_turn_velocity,
                max_steering_rad=stanley_max_steering_rad,
            ),
        )

        bl.node(
            "svea_charging",
            "line_follower.py",
            name="line_follower",
            params=dict(
                use_rviz=use_foxglove,
                is_sim=False,
                image_topic=camera_image_topic,
                aruco_stop_distance_m=bt_dock_distance_m,
            ),
        )

        bl.node(
            "svea_charging",
            "bt_runner.py",
            name="bt_runner",
            params=dict(
                switch_distance_m=bt_switch_distance_m,
                docking_exit_distance_m=bt_docking_exit_distance_m,
                dock_distance_m=bt_dock_distance_m,
                charge_start_voltage=bt_charge_start_voltage,
                charge_done_voltage=bt_charge_done_voltage,
                charge_voltage_confirm_s=bt_charge_voltage_confirm_s,
            ),
        )

        bl.node(
            "svea_charging",
            "control_mux.py",
            name="control_mux",
            params=dict(
                is_sim=False,
                controller_timeout_s=control_mux_timeout_s,
            ),
        )
