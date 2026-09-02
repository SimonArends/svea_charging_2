#!/usr/bin/env python3
"""Apply v4l2 controls that usb_cam_node's own params can't reach on this
camera (see LINE_FOLLOWER_CAMERA_HANDOFF.md problem 1: the node's
auto_white_balance/autoexposure params map to control names this driver
doesn't expose, so they silently no-op and the camera stays in its default
auto-exposure mode). Runs the equivalent v4l2-ctl --set-ctrl calls directly
and exits; not a long-running node.
"""

import subprocess

from svea_core import rosonic as rx


class set_camera_ctrls(rx.Node):
    video_device = rx.Parameter("/dev/video0")
    auto_exposure = rx.Parameter(1)  # 1 = Manual Mode (UVC menu control)
    exposure_time_absolute = rx.Parameter(150)
    white_balance_automatic = rx.Parameter(0)
    white_balance_temperature = rx.Parameter(4500)

    def run(self):
        ctrls = {
            "auto_exposure": self.auto_exposure,
            "exposure_time_absolute": self.exposure_time_absolute,
            "white_balance_automatic": self.white_balance_automatic,
            "white_balance_temperature": self.white_balance_temperature,
        }
        for name, value in ctrls.items():
            cmd = [
                "v4l2-ctl",
                "-d",
                self.video_device,
                f"--set-ctrl={name}={value}",
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().warn(
                    f"Failed to set {name}={value} on {self.video_device}: "
                    f"{result.stderr.strip()}"
                )
            else:
                self.get_logger().info(f"Set {name}={value} on {self.video_device}")


if __name__ == "__main__":
    set_camera_ctrls.main()
