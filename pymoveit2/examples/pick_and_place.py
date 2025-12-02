#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Locks the detected color coordinates before starting the motion.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

"""

from threading import Thread
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        # Flags
        self.already_moved = False
        self.target_coords = None  # Stores the locked coordinates

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.05
        self.moveit2.max_acceleration = 0.05

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_controller/follow_joint_trajectory",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")

        # Predefined joint positions (in radians)
        # Start in a ready pose within joint limits
        self.start_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self._do_motion("move to start", self.moveit2.move_to_configuration, self.start_joints)
        self._do_wait("wait start", self.moveit2.wait_until_executed)

    def coords_callback(self, msg):
        if self.already_moved:
            return  # Ignore messages once motion starts

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                # Lock coordinates immediately
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
                )
                self.already_moved = True

                # Use locked coordinates
                side_offset = -0.001
                pick_position = [self.target_coords[0], self.target_coords[1], self.target_coords[2] - 0.65]  # was 0.60

                quat_xyzw = [0.0, 1.0, 0.0, 0.0]

                # --- Pick-and-place sequence ---

                # 1. Move to home joint configuration
                self._do_motion("home configuration", self.moveit2.move_to_configuration, self.home_joints)
                self._do_wait("wait home", self.moveit2.wait_until_executed)

                # 2. Move above target (Cartesian)
                self._do_motion("above target", self.moveit2.move_to_pose, position=pick_position, quat_xyzw=quat_xyzw)
                self._do_wait("wait above target", self.moveit2.wait_until_executed)

                # 3. Open gripper (ensure fully open)
                self._do_motion("gripper open", self.gripper.open)
                self._do_wait("wait gripper open", self.gripper.wait_until_executed)
                self._do_motion("gripper open retry", self.gripper.open)
                self._do_wait("wait gripper open retry", self.gripper.wait_until_executed)
                time.sleep(0.2)

                # 4. Move down to approach object

                approach_position = [pick_position[0], pick_position[1], pick_position[2] - 0.28]
                self._do_motion("approach target", self.moveit2.move_to_pose, position=approach_position, quat_xyzw=quat_xyzw)
                self._do_wait("wait approach", self.moveit2.wait_until_executed)

                # 5. Close gripper
                self._do_motion("gripper close", self.gripper.close)
                self._do_wait("wait gripper close", self.gripper.wait_until_executed)
                self._do_motion("gripper close retry", self.gripper.close)
                self._do_wait("wait gripper close retry", self.gripper.wait_until_executed)
                #rclpy.sleep(Duration(seconds=0.3))  # brief hold if you prefer, or time.sleep

                # Lift slightly after close to reduce slip
                lift_position = [pick_position[0], pick_position[1], pick_position[2] + 0.05]
                self._do_motion("lift after close", self.moveit2.move_to_pose, position=lift_position, quat_xyzw=quat_xyzw)
                self._do_wait("wait lift after close", self.moveit2.wait_until_executed)


                # 7. Move to home joint configuration
                self._do_motion("home after pick", self.moveit2.move_to_configuration, self.home_joints)
                self._do_wait("wait home after pick", self.moveit2.wait_until_executed)

                # 8. Move to drop joint configuration
                self._do_motion("move to drop", self.moveit2.move_to_configuration, self.drop_joints)
                self._do_wait("wait drop", self.moveit2.wait_until_executed)

                # 9. Open gripper to release
                self._do_motion("gripper open drop", self.gripper.open)
                self._do_wait("wait gripper open drop", self.gripper.wait_until_executed)

                # 10. Close gripper
                self._do_motion("gripper close end", self.gripper.close)
                self._do_wait("wait gripper close end", self.gripper.wait_until_executed)

                # 11. Return to start joint configuration
                self._do_motion("return to start", self.moveit2.move_to_configuration, self.start_joints)
                self._do_wait("wait return to start", self.moveit2.wait_until_executed)

                self.get_logger().info("Pick-and-place sequence complete.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")

    def _do_motion(self, label, fn, *args, **kwargs):
        self.get_logger().info(f"Starting motion: {label}")
        try:
            fn(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f"{label} failed to start: {e}")

    def _do_wait(self, label, fn):
        try:
            result = fn()
            self.get_logger().info(f"Completed: {label} (result={result})")
        except Exception as e:
            self.get_logger().error(f"{label} failed: {e}")


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
