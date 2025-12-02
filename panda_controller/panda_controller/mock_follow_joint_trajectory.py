#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import threading
import time

class MockFollowJointTrajectory(Node):
    def __init__(self):
        super().__init__('mock_follow_joint_trajectory')

        # Parameters
        self.declare_parameter('arm_joints', [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ])
        self.declare_parameter('gripper_joints', [
            'panda_finger_joint1', 'panda_finger_joint2'
        ])
        self.declare_parameter('publish_rate_hz', 30.0)

        self.arm_joints = list(self.get_parameter('arm_joints').value)
        self.gripper_joints = list(self.get_parameter('gripper_joints').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        # State
        self.lock = threading.Lock()
        self.positions = {
            'panda_joint1': 0.0,
            'panda_joint2': -0.785,
            'panda_joint3': 0.0,
            'panda_joint4': -2.356,
            'panda_joint5': 0.0,
            'panda_joint6': 1.571,
            'panda_joint7': 0.785,
            'panda_finger_joint1': 0.02,
            'panda_finger_joint2': 0.02
        }

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # Action servers for arm and gripper
        self.arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_cb,
        )
        self.gripper_action = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_cb,
        )

        self.get_logger().info('Mock controllers ready (arm_controller, gripper_controller)')

    def publish_joint_states(self):
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.positions.keys())
            msg.position = [self.positions[n] for n in msg.name]
        self.joint_state_pub.publish(msg)

    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Take the last point as goal; update positions immediately.
        last = traj.points[-1]
        joint_names = list(traj.joint_names)
        positions = list(last.positions)

        with self.lock:
            for name, pos in zip(joint_names, positions):
                self.positions[name] = float(pos)

        # Simulate small execution time
        time.sleep(0.05)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ''
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockFollowJointTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
