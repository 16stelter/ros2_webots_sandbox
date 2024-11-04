#!/usr/bin/env python3
import subprocess
import threading

import argparse

import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node


class WebotsSim(Node):

    def __init__(self, nogui, multi_robot, headless, sim_port):
        super().__init__('ros2_webots_sandbox')
        pkg_path = get_package_share_directory("ros2_webots_sandbox")

        # construct arguments with which webots is started depending on this scripts args
        extra_args = set()
        mode = "realtime"
        if nogui:
            mode = "fast"
            extra_args.update(["--batch", "--no-rendering"])

        if multi_robot:
            world_name = "terrain_example.wbt"
        else:
            world_name = "terrain_example_single.wbt"

        if headless:
            cmd = [
                "xvfb-run",
                "--auto-servernum",
                "webots",
            ]
            extra_args.update(["--stdout", "--stderr", "--batch", "--no-rendering"])
            mode = "fast"
        else:
            cmd = ["webots"]

        # actually start webots
        cmd_with_args = cmd + list(extra_args) + [f"--mode={mode}", f"--port={sim_port}", f"{pkg_path}/worlds/{world_name}"]
        print(f"running {' '.join(cmd_with_args)}")
        self.sim_proc = subprocess.Popen(cmd_with_args)

    def run_simulation(self):
        # join with child process
        try:
            exit(self.sim_proc.wait())
        except KeyboardInterrupt:
            exit(self.sim_proc.returncode)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--headless',
                       help='Starts webots completely headless on a virtual framebuffer. This also automatically enables --batch and --no-rendering for webots',
                       action='store_true')
    group.add_argument('--nogui', help="Deactivate gui", action='store_true')
    parser.add_argument('--sim-port', help="port of the simulation", default="1234")
    parser.add_argument('--multi-robot', help="start world with a single robot", action='store_true')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WebotsSim(args.nogui, args.multi_robot, args.headless, args.sim_port)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run_simulation()

    node.destroy_node()
    rclpy.shutdown()
