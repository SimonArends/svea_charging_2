#!/usr/bin/env python3

from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    use_datum: bool = True,
    datum_file: str = "",
    rtk_device: str = "/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR_WiFi_1010_C5EE644B5150484347202020FF0E0B39-if00",
    rtk_baud: int = 115200,
    rtk_username: str = "ITRL03",
    rtk_password: str = "171488",
):

    bl = BetterLaunch()
    if not datum_file:
        datum_file = bl.find("svea_charging", "params/outdoor_datum.yaml")

    bl.include(
        "svea_core",
        "svea.launch.py",
        name="svea_a",
        is_sim=False,
        use_map=False,
        use_foxglove=False,
        is_indoor=False,
        use_localization=True,
        use_rtk=True,
        rtk_device=rtk_device,
        rtk_baud=rtk_baud,
        rtk_username=rtk_username,
        rtk_password=rtk_password,
        use_datum=use_datum,
        datum_service="datum",
        datum_file=datum_file,
    )

    bl.node(
        "svea_charging",
        "signal_logger.py",
        name="signal_logger",
    )