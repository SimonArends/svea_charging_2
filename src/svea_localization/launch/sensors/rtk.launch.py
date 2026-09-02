#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    ## RTK RECEIVER ARGUMENTS
    device: str = "/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR_WiFi_1010_C5EE644B5150484347202020FF0E0B39-if00",
    baud: int = 115200,                     # 38400 if connected via UART
    receiver_interface: str = "uart1",
    gps_frame: str = "gps",
    dynamic_model: str = "portable",
    ## NTRIP CLIENT ARGUMENTS (for swepos network rtk)
    host: str = "nrtk-swepos.lm.se",
    port: int = 80,                         # PORT 8500 is also valid
    authenticate: bool = True,
    mountpoint: str = "MSM_GNSS",
    ntrip_namespace: str = "gps",
    username: str = "",
    password: str = ""
):
    
    bl = BetterLaunch()

    # The included NTRIP launch file parses launch arguments as YAML. Quote the
    # password explicitly so a numeric-only password remains a string.
    password_parameter = "'" + password.replace("'", "''") + "'"

    with bl.group("gps"):

        # Start RTK Manager Node
        bl.node("svea_localization", "rtk_manager.py",
                name="rtk_manager",
                params=dict(device=device,
                            baud=baud,
                            receiver_interface=receiver_interface,
                            gps_frame=gps_frame,
                            dynamic_model=dynamic_model))

    # Start NTRIP Client
    bl.include("ntrip_client", "ntrip_client_launch.py",
               namespace=ntrip_namespace,
               host=host,
               port=port,
               mountpoint=mountpoint,
               authenticate=authenticate,
               username=username,
               password=password_parameter)
