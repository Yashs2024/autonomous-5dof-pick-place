#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # Connect directly to the active Gazebo ros2_control trajectory servers
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_group_controller/follow_joint_trajectory'
        )

        self.arm_joints = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'
        ]
        self.gripper_joints = [
            'gripper_left_joint', 'gripper_right_joint'
        ]

    def move_arm(self, positions, duration_sec=2.5):
        self.get_logger().info(f'Moving arm to: {positions}')
        self.arm_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal_msg.trajectory.points = [point]

        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        time.sleep(0.3)
        return True

    def move_gripper(self, jaw_pos, duration_sec=1.0):
        # Both fingers share equal limits [-0.250, 0.6] with opposite axes in URDF
        self.get_logger().info(f'Setting gripper jaws to: {jaw_pos}')
        self.gripper_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()
        point.positions = [float(jaw_pos), float(jaw_pos)]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal_msg.trajectory.points = [point]

        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper trajectory rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        time.sleep(0.3)
        return True

    def execute_sequence(self):
        self.get_logger().info('--- Starting Gazebo Pick and Place Sequence ---')

        # 1. Move to Home & Open Gripper Wide (0.45 rad)
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=2.0)
        self.move_gripper(0.45, duration_sec=1.0)

        # 2. Hover above the Red Cube at (x=0.18, y=0.0)
        self.move_arm([0.0, 0.55, 0.65, 0.0, 0.35], duration_sec=2.5)

        # 3. Lower Jaws around the Red Cube
        self.move_arm([0.0, 0.78, 0.82, 0.0, 0.15], duration_sec=2.0)

        # 4. Close Gripper Tight (-0.20 rad) to Grasp Cube
        self.move_gripper(-0.22, duration_sec=1.2)

        # 5. Lift the Cube Up
        self.move_arm([0.0, 0.35, 0.45, 0.0, 0.20], duration_sec=3.5)

        # 6. Rotate Waist (joint_1) to Drop-off Zone (y > 0)
        self.move_arm([1.2, 0.45, 0.55, 0.0, 0.20], duration_sec=4.0)

        # 7. Lower Slightly & Open Gripper to Release Cube
        self.move_arm([1.2, 0.72, 0.78, 0.0, 0.15], duration_sec=2.0)
        self.move_gripper(0.45, duration_sec=1.0)

        # 8. Return to Home Pose
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=2.5)
        self.move_gripper(0.0, duration_sec=1.0)
        self.get_logger().info('--- Gazebo Pick and Place Sequence Complete! ---')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    node.execute_sequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
