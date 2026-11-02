#!/usr/bin/env python3

import argparse
import sys

import rclpy
from gazebo_msgs.srv import SetModelConfiguration


DOG_JOINT_NAMES = [
    "rotate_left_front_shoulder_joint",
    "rotate_left_front_thigh_joint",
    "rotate_left_front_shank_joint",
    "rotate_left_after_shoulder_joint",
    "rotate_left_after_thigh_joint",
    "rotate_left_after_shank_joint",
    "rotate_right_front_shoulder_joint",
    "rotate_right_front_thigh_joint",
    "rotate_right_front_shank_joint",
    "rotate_right_after_shoulder_joint",
    "rotate_right_after_thigh_joint",
    "rotate_right_after_shank_joint",
]
DOG_INIT_ANGLE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.018, 0.0, 0.0, 0.0, 0.0, 0.0, 0.057]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Set the dog reset joint positions in Gazebo")
    parser.add_argument("--model-name", default="dog")
    parser.add_argument("--service", default="/gazebo/set_model_configuration")
    cli_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=cli_args)[1:])

    rclpy.init()
    node = rclpy.create_node("set_initial_joint_positions")
    try:
        client = node.create_client(SetModelConfiguration, args.service)
        if not client.wait_for_service(timeout_sec=30.0):
            node.get_logger().error(f"Timed out waiting for {args.service}")
            return 1

        request = SetModelConfiguration.Request()
        request.model_name = args.model_name
        request.urdf_param_name = "robot_description"
        request.joint_names = DOG_JOINT_NAMES
        request.joint_positions = DOG_INIT_ANGLE
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if not future.done() or future.exception() is not None:
            node.get_logger().error(f"Failed to call {args.service}: {future.exception()}")
            return 1
        response = future.result()
        if not response.success:
            node.get_logger().error(f"Gazebo rejected initial joint positions: {response.status_message}")
            return 1
        node.get_logger().info(f"Set {len(DOG_JOINT_NAMES)} initial joint positions for {args.model_name}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
