#!/usr/bin/env python3

from better_launch import BetterLaunch, launch_this

ROUTE_PRESETS = {
    #route and parameters have been adjusted for simulation. Now contains routes from point A (-1.2, 0) and B (1.2, 0) the origin. 
    "charging_station": dict(
        route_config="params/routes/to_charging_station.yaml",
        aruco_marker_id=11,
    ),
}


@launch_this
def main(
    is_sim: bool = True,
    name: str = "self",
    enabled: bool = True,
    transport_start_location: str = "A",
    initial_pose_x: float = -1.2,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 1.5,
    route_preset: str = "charging_station",
    route_config: str = "",
    use_datum: bool = True,
    datum_file: str = "",
    rtk_device: str = "/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR_WiFi_1010_C5EE644B5150484347202020FF0E0B39-if00",
    rtk_baud: int = 115200,
    rtk_username: str = "ITRL03",
    rtk_password: str = "171488",
    use_foxglove: bool = False,  #node is launched seperately when launching 2 SVEA's, this is the reason to have it false in general
    use_gps: bool = False,
    docking_target_velocity: float = 0.09,
    dock_target_angle_deg: float = 85.0,
    bt_dock_distance_m: float = 0.6251276731491089,
    bt_switch_distance_m: float = 1.0, #the aruco is placed right in the charging point (origin) in simulation. An actual aruco would need to be somewhere else.
    bt_docking_exit_distance_m: float = 2.75,
    bt_charge_start_voltage: float = 12.45, #lower gives more trips in transport mode, if you are close to charge done voltage you might always be charging.
    bt_charge_done_voltage: float = 12.55,
    bt_charge_voltage_confirm_s: float = 3.0,
    stanley_target_velocity: float = 0.48, #gives approx 0.33 velocity commands by the Stanley. Stanley does not reach target. 
    stanley_turn_velocity: float = 0.3, 
    stanley_max_steering_rad: float = 0.35, #something to adjust?
    control_mux_timeout_s: float = 1.0,
    post_a_params = "",
    transport_stanley_params = "",
    # Map
    use_map: bool = True,
    map_pkg: str = 'svea_core',
    map_name: str = 'floor2',
    map_topic: str = '/map',
    battery_charge_current: float = 19.5,
    battery_discharge_current_stationary: float = -0.9,
    battery_discharge_current_driving: float = -1.8,
):

    bl = BetterLaunch()

    if route_preset:
        if route_preset not in ROUTE_PRESETS:
            raise ValueError(
                f"Unknown route_preset '{route_preset}', expected one of "
                f"{list(ROUTE_PRESETS)}"
            )
        preset = ROUTE_PRESETS[route_preset]
        if not route_config:
            route_config = bl.find("svea_charging", preset["route_config"])

    if not route_config:
        route_config = bl.find("svea_charging", "params/routes/to_parking_lot.yaml")
    if not datum_file:
        datum_file = bl.find("svea_charging", "params/outdoor_datum.yaml")
    if not post_a_params:
        post_a_params = bl.find("svea_charging", "params/routes/post_a.yaml")
    if not transport_stanley_params:
        transport_stanley_params = bl.find("svea_charging", "params/routes/transport_stanley.yaml")
    
    bl.node("nav2_map_server", "map_server",
                name="map_server",
                params=dict(yaml_filename=bl.find(map_pkg, f"{map_name}.yaml"),
                            use_sim_time=False,
                            topic_name=map_topic))

    bl.include("foxglove_bridge", "foxglove_bridge_launch.xml",
                   port=8765)

    bl.node(
        "svea_charging",
        "automatic_scheduler.py",
        name="automatic_scheduler",
        params=dict(
        charge_done_voltage = bt_charge_done_voltage,
        charging_current = battery_charge_current,
        idle_current = battery_discharge_current_stationary,
        moving_current = battery_discharge_current_driving,
        ),
    )

    bl.node(
        "svea_charging",
        "performance_logger.py",
        name="performance_logger",
    )

    INITIAL_POSES = {
    "svea_a": (-1.2, 0.0, 1.5, "A", "svea_b"),
    "svea_b": (1.2, 0.0, -1.64, "B", "svea_a"),
    }

    for name, (init_x, init_y, init_a, start_loc, other_name) in INITIAL_POSES.items():

        bl.include(
            "svea_core",
            "svea.launch.py",
            name=name,
            is_sim=is_sim,
            is_indoor=False,
            initial_pose_x=init_x,
            initial_pose_y=init_y,
            initial_pose_a=init_a,
            use_localization=True,
            use_map=False,
            map_name = map_name, #the digital cylinders are launched in "floor2", the rest is not used.
            use_rtk=use_gps,
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
                    **{"localization/base_frame": f"{name}/base_link"},
                ),
            )

            bl.node(
                "svea_charging",
                "transport_stanley.py",
                name="transport_stanley",
                param_files=transport_stanley_params,
                params=dict(
                    enabled=enabled,
                    controller_name="transport_stanley",
                    target_velocity=stanley_target_velocity,
                    turn_velocity=stanley_turn_velocity,
                    max_steering_rad=stanley_max_steering_rad,
                    location=start_loc,
                    **{"localization/base_frame": f"{name}/base_link"},
                ),
            )

            bl.node(
                "svea_charging",
                "post_stanley.py",
                name="post_stanley",
                param_files=post_a_params,
                params=dict(
                    enabled=enabled,
                    controller_name="post_stanley",
                    target_velocity=stanley_target_velocity,
                    turn_velocity=stanley_turn_velocity,
                    max_steering_rad=stanley_max_steering_rad,
                    **{"localization/base_frame": f"{name}/base_link"},
                ),
            )

            bl.node(
                "svea_charging",
                "cylinder_docking.py",
                name="cylinder_docking",
                params={
                    "is_sim": is_sim,
                    "scan_topic": "scan",
                    "target_velocity": docking_target_velocity,
                    "dock_target_angle_deg": dock_target_angle_deg,
                    "localization/base_frame": f"{name}/base_link",
                },
            )

            bl.node(
                "svea_charging",
                "bt_runner.py",
                name="bt_runner",
                params=dict(
                    switch_distance_m=bt_switch_distance_m,
                    docking_exit_distance_m=bt_docking_exit_distance_m,
                    charge_start_voltage=bt_charge_start_voltage,
                    charge_done_voltage=bt_charge_done_voltage,
                    charge_voltage_confirm_s=bt_charge_voltage_confirm_s,
                ),
            )

            bl.node(
                "svea_charging",
                "cylinder_control_mux.py",
                name="cylinder_control_mux",
                params=dict(
                    name_svea = name,
                    other_svea = other_name,        
                    controller_timeout_s=control_mux_timeout_s,
                    is_sim = is_sim,
                    **{"localization/base_frame": f"{name}/base_link"},
                ),
            )
            
            if is_sim:
                bl.node(
                    "svea_charging",
                    "battery_simulator.py",
                    name="battery_simulator",
                    params=dict(
                        battery_charge_current=battery_charge_current,
                        battery_discharge_current_stationary = battery_discharge_current_stationary,        
                        battery_discharge_current_driving = battery_discharge_current_driving,
                    ),
                )
