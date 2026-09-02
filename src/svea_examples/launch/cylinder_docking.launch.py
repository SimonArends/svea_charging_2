#! /usr/bin/env python3
from better_launch import BetterLaunch, launch_this

MAP_NAME = "floor2"

@launch_this
def main(
    is_sim: bool = True,
    use_foxglove: bool = True,
    initial_pose_x: float = -7.4,
    initial_pose_y: float = -15.4,
    initial_pose_a: float = +0.9,
    target_velocity: float = 0.4,
    dock_target_angle_deg: float = 85.0,
):
    bl = BetterLaunch()

    if not is_sim:

        # Start SVEA in real-world mode with LiDAR enabled for docking
        bl.include(
            "svea_core", 
            "svea.launch.py",
            is_sim=is_sim, # False
            map_name=MAP_NAME,
            initial_pose_x=initial_pose_x,
            initial_pose_y=initial_pose_y,
            initial_pose_a=initial_pose_a,
            use_lidar=True,
        )

        with bl.group("self"):
            bl.node(
                "svea_charging", 
                "cylinder_docking.py",
                name="cylinder_docking",
                params={
                    "scan_topic": "scan",
                    "target_velocity": target_velocity,
                    "dock_target_angle_deg": dock_target_angle_deg,
                    "localization/base_frame": "self/base_link",
                },
            )

    if is_sim:
        INITIAL_POSES = {
            "svea_a": (0.0, 0.0, 0.0),
        }

        for name, (init_x, init_y, init_a) in INITIAL_POSES.items():
            
            bl.include(
                "svea_core", 
                "svea.launch.py",
                name=name,
                is_sim=is_sim,
                is_indoor=True,
                map_name=MAP_NAME,
                initial_pose_x=init_x,
                initial_pose_y=init_y,
                initial_pose_a=init_a,
                use_lidar=True,
            )

            # Namespace scan topic and localization frame for each vehicle
            with bl.group(name):
                bl.node(
                    "svea_charging", 
                    "cylinder_docking.py",
                    name="cylinder_docking",
                    params={
                        "scan_topic": "scan",
                        "target_velocity": target_velocity,
                        "dock_target_angle_deg": dock_target_angle_deg,
                        "localization/base_frame": f"{name}/base_link",
                    },
                )

    bl.include(
        "svea_core", 
        "map_and_foxglove.launch.py",
        map_name=MAP_NAME,
        use_foxglove=use_foxglove,
    )