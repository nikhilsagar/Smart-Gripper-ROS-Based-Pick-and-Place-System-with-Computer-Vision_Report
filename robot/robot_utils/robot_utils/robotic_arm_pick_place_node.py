import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import subprocess
import time

class RoboticArmPickPlaceNode(Node):
    def __init__(self):
        super().__init__('robotic_arm_pick_place_node')

        self.subscription = self.create_subscription(
            String,
            '/top_priority_object',
            self.listener_callback,
            10
        )

        self.declare_and_load_json()
        self.send_grip_open()
        self.send_to_robot(*self.HOME_POSITION)

        self.get_logger().info("🟢 Robotic arm initialized at HOME with grip open.")

    def declare_and_load_json(self):
        self.json_file = 'verified_coordinates.json'
        if os.path.exists(self.json_file):
            with open(self.json_file, 'r') as f:
                self.verified_coords = json.load(f)
        else:
            self.get_logger().error("❌ verified_coordinates.json not found.")
            self.verified_coords = {}

        self.HOME_POSITION = [0.01, 0.10, 0.30, 0.0]

        self.DROP_POSITIONS = {
            "Red": {
                "static": [0.25, 0.1, 0.1, 0.0],
                "drop":   [0.25, 0.1, 0.1, -1.0]
            },
            "Green": {
                "static": [0.3, 0.3, 0.1, 0.0],
                "drop":   [0.3, 0.3, 0.1, -1.0]
            },
            "Blue": {
                "static": [-0.25, 0.1, 0.1, 0.0],
                "drop":   [-0.25, 0.1, 0.1, -1.0]
            },
            "Yellow": {
                "static": [-0.3, 0.3, 0.1, 0.0],
                "drop":   [-0.3, 0.3, 0.1, -1.0]
            }
        }

    def listener_callback(self, msg):
        try:
            # Parse message: "ID: 21, Color: Red"
            text = msg.data.strip()
            parts = [p.strip() for p in text.split(',')]
            id_str = parts[0].split(':')[1].strip()
            color = parts[1].split(':')[1].strip().capitalize()

            self.get_logger().info(f"📥 Received target: ID={id_str}, Color={color}")

            if id_str not in self.verified_coords:
                self.get_logger().warn(f"⚠️ ID {id_str} not found in verified_coordinates.json")
                return

            pick_position = self.verified_coords[id_str]
            drop_static = self.DROP_POSITIONS[color]["static"]
            drop_final = self.DROP_POSITIONS[color]["drop"]

            # 🟡 Pick & Place Routine
            self.send_to_robot(*pick_position)
            self.send_grip_close()
            time.sleep(1)

            self.send_to_robot(*self.HOME_POSITION)
            time.sleep(1)

            self.send_to_robot(*drop_static)
            self.send_grip_open()
            time.sleep(1)

            self.send_to_robot(*drop_final)
            time.sleep(1)

            self.send_to_robot(*self.HOME_POSITION)
            time.sleep(1)

            self.get_logger().info("✅ Completed pick-and-place cycle.")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to execute pick-place sequence: {e}")

    def send_to_robot(self, x, y, z, angle):
        command = (
            f"ros2 action send_goal /follow_ik_trajectory control_msgs/action/FollowJointTrajectory "
            f"\"{{trajectory: {{ points: [{{positions: [{x}, {y}, {z}, {angle}]}}]}}}}\""
        )
        self.get_logger().info(f"📤 Moving to: {x}, {y}, {z}, {angle}")
        subprocess.run(command, shell=True)

    def send_grip_open(self):
        command = (
            "ros2 action send_goal /gripper_controller/follow_joint_trajectory "
            "control_msgs/action/FollowJointTrajectory "
            "\"{trajectory: { joint_names: ['joint_5'], points: [{positions: [-0.03], time_from_start: {sec: 1}}]}}\""
        )
        self.get_logger().info("👐 Opening gripper...")
        subprocess.run(command, shell=True)

    def send_grip_close(self):
        command = (
            "ros2 action send_goal /gripper_controller/follow_joint_trajectory "
            "control_msgs/action/FollowJointTrajectory "
            "\"{trajectory: { joint_names: ['joint_5'], points: [{positions: [0.0], time_from_start: {sec: 1}}]}}\""
        )
        self.get_logger().info("✊ Closing gripper...")
        subprocess.run(command, shell=True)


def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmPickPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
