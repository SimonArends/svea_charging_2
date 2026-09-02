#!/usr/bin/env python3

from better_launch import BetterLaunch, launch_this


@launch_this
def main(
    name: str = "self",
    enabled: bool = False,
    initial_pose_a: float = 0.0,
    route_config: str = "",
    rtk_device: str = "/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR_WiFi_1010_C5EE644B5150484347202020FF0E0B39-if00",
    rtk_baud: int = 115200,
    rtk_username: str = "ITRL03",
    rtk_password: str = "171488",
    use_foxglove: bool = True,
):
    """Start only the hardware, outdoor localization, and outdoor follower."""
    bl = BetterLaunch()

    if not route_config:
        route_config = bl.find("svea_charging", "params/outdoor_route.yaml")

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
        use_foxglove=use_foxglove,
    )

    with bl.group(name):
        bl.node(
            "svea_charging",
            "outdoor_stanley.py",
            name="outdoor_stanley",
            param_files=route_config,
            params={
                "enabled": enabled,
            },
        )
