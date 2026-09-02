#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
    is_sim: bool = True,
    is_indoor: bool = True,
    initial_pose_x: float = 0.0,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 0.0,
    # Map
    use_map: bool = True,
    map_pkg: str = 'svea_core',
    map_name: str = 'sml',
    map_topic: str = '/map',
    # Coordinate Frames
    map_frame: str = 'map',
    odom_frame: str = '{name}/odom',
    base_frame: str = '{name}/base_link',
    # LiDAR Settings
    use_lidar: bool = True,
    lidar_ip: str = '192.168.0.10',
    # RTK-GPS Settings
    use_rtk: bool = True,
    rtk_device: str = '/dev/ttyACM1',
    rtk_baud: int = 115200,
    rtk_username: str = '',
    rtk_password: str = '',
    # Datum Settings
    use_datum: bool = False,
    datum_service: str = 'datum',
    datum_file: str = '',
    datum_data: str = '[]',
):
    bl = BetterLaunch()

    # Format the coordinate frames with the robot name
    map_frame = map_frame.format(name=name)
    odom_frame = odom_frame.format(name=name)
    base_frame = base_frame.format(name=name)

    # MAVROS data_raw provides angular velocity but no absolute orientation.
    # Seed both filters with an earth-referenced ENU yaw and integrate yaw rate.
    # initial_pose_a is in radians: 0=east, pi/2=north.
    ekf_initial_state = [
        0.0, 0.0, 0.0,
        0.0, 0.0, initial_pose_a,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ]
    if use_map:

        bl.node("nav2_map_server", "map_server",
                name="map_server",
                params=dict(yaml_filename=bl.find(map_pkg, f"{map_name}.yaml"),
                            use_sim_time=False,
                            topic_name=map_topic))

    with bl.group(name):

        # Static Transforms
        bl.include("svea_localization", "transforms.launch.py",
                   name=name,
                   use_gps=use_rtk,
                   use_lidar=use_lidar,
                   map_frame=map_frame,
                   odom_frame=odom_frame,
                   base_frame=base_frame)

    if is_sim:
        # Currently no localization is needed in simulation, as the simulator
        # provides perfect odometry and pose information. However, this section
        # can be expanded in the future to include simulated sensors and
        # localization algorithms if desired.
        pass

    else:

        USE_SIM_TIME = False

        # Load default parameters
        # Outdoors we no longer fuse the IMU in the local EKF (RTK-safe Stanley
        # navigation relies on GPS/encoder-only local odometry).
        local_ekf_file = "local_ekf.yaml" if is_indoor else "local_ekf_outdoors.yaml"
        LOCAL_EKF_PARAMS = bl.find("svea_localization", local_ekf_file)
        GLOBAL_EKF_PARAMS = bl.find("svea_localization", "global_ekf.yaml")
        AMCL_PARAMS = bl.find("svea_localization", "amcl.yaml")

        with bl.group(name):
            bl.node("robot_localization", "ekf_node",
                    name="ekf_local",
                    param_files=LOCAL_EKF_PARAMS,
                    params={"map_frame": map_frame,
                            "odom_frame": odom_frame,
                            "base_link_frame": base_frame,
                            "world_frame": odom_frame,
                            "initial_state": ekf_initial_state},
                    remaps={"odometry/filtered": "odometry/local"})

        if use_lidar:
            with bl.group(name):
                bl.include("svea_localization", "lidar.launch.py",
                           lidar_ip=lidar_ip,
                           lidar_frame=f"{name}/laser")

        if is_indoor:

            with bl.group(name):
                # BetterLaunch manages lifecycle nodes automatically, so no need to
                # run lifecycle_manager manually.
                bl.node("nav2_amcl", "amcl",
                        name="amcl",
                        param_files=AMCL_PARAMS,
                        params={"use_sim_time": USE_SIM_TIME,
                                "yaml_filename": bl.find(map_pkg, f"{map_name}.yaml"),
                                "initial_pose_x": initial_pose_x,
                                "initial_pose_y": initial_pose_y,
                                "initial_pose_a": initial_pose_a,
                                "base_frame_id": base_frame,
                                "odom_frame_id": odom_frame,
                                "map_frame_id": map_frame})

        elif use_rtk:

            with bl.group(name):
                bl.include("svea_localization", "rtk.launch.py",
                           device=rtk_device,
                           baud=rtk_baud,
                           gps_frame=f"{name}/gps",
                           ntrip_namespace=f"{name}/gps",
                           username=rtk_username,
                           password=rtk_password)
                    
                # Start NavSat Transform Node
                bl.node("robot_localization", "navsat_transform_node",
                        name="navsat_transform_node",
                        params=dict(publish_filtered_gps=True,
                                    wait_for_datum=use_datum,
                                    delay=2.0,
                                    magnetic_declination_radians=0.0,
                                    yaw_offset=0.0,
                                    use_odometry_yaw=True,
                                    zero_altitude=True,
                                    broadcast_cartesian_transform_as_parent_frame=True,
                                    broadcast_cartesian_transform=True),
                        remaps={"gps/fix": "gps/fix",
                                "gps/filtered": "gps/filtered",
                                "odometry/gps": "odometry/gps",
                                "odometry/filtered": "odometry/global"})

                # Start Set Datum Node
                if use_datum:
                    bl.node("svea_localization", "set_datum_node.py",
                            name="set_datum_node",
                            params=dict(datum_service=datum_service,
                                        service_timeout=60.0,
                                        datum_file=datum_file,
                                        datum_data=datum_data))

                bl.node("robot_localization", "ekf_node",
                        name="ekf_global",
                        param_files=GLOBAL_EKF_PARAMS,
                        params={"map_frame": map_frame,
                                "odom_frame": odom_frame,
                                "base_link_frame": base_frame,
                                "world_frame": map_frame,
                                "initial_state": ekf_initial_state},
                        remaps={"odometry/filtered": "odometry/global"})
