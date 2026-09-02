#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
from typing import Literal

@launch_this
def main(
    joy_kind: str = 'xbox',
    use_joy: bool = False,
):
    bl = BetterLaunch()

    # Start SVEA system
    bl.include("svea_core", "svea.launch.py",
               is_sim=False)

    # Start Joy->SVEA translator
    if use_joy:
        bl.node("svea_examples", "joy_consumer.py",
                name="joy_consumer",
                params=dict(joy_top="/joy",
                            joy_kind=joy_kind,
                            joy_btns=','.join([
                                "START:/qod",
                                "BACK:/load_status",
                                "DPADU:/load_on",
                                "DPADD:/load_off",
                            ] if joy_kind == 'xbox' else [
                                "ENTER:/qod",
                                "SHARE:/load_status",
                                "PLUS:/load_on",
                                "MINUS:/load_off",
                            ] if joy_kind == 'g29' else [])))
    else:
        bl.node("svea_examples", "twist_consumer.py",
                name="twist_consumer",
                params=dict(twist_top="fmq/remote_control",
                            twist_type="geometry_msgs/msg/TwistStamped"))

