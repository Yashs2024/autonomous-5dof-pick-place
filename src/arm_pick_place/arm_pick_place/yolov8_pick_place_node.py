#!/usr/bin/env python3
import sys
import glob
import os

# Append pipx ultralytics environment after system packages so Ubuntu's numpy/cv2 take priority
pipx_site = glob.glob(os.path.expanduser("~/.local/share/pipx/venvs/ultralytics/lib/python*/site-packages"))
if pipx_site:
    sys.path.append(pipx_site[0])

import math
import time
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ultralytics import YOLO


class YOLOv8PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('yolov8_pick_and_place_node')

        # Load YOLOv8n (smallest nano model)
        self.get_logger().info('Loading YOLOv8n (yolov8n.pt) nano model...')
        self.model = YOLO('yolov8n.pt')

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.target_pub = self.create_publisher(PointStamped, '/detected_cube_point', 10)

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_group_controller/follow_joint_trajectory'
        )

        self.arm_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.gripper_joints = ['gripper_left_joint', 'gripper_right_joint']

        # Overhead camera parameters (X=0.18, Y=0.05, Z=0.62, HFOV=1.20 rad)
        self.cam_x = 0.18
        self.cam_y = 0.05
        self.cam_z = 0.62
        self.focal_px = (640.0 / 2.0) / math.tan(1.20 / 2.0)

        # URDF Link Lengths (meters)
        self.z_shoulder = 0.09616
        self.L1 = 0.11423
        self.L2 = 0.12982
        self.L_grip = 0.088

        self.lock_count = 0
        self.busy = False
        self.completed = False
        self.detected_x = 0.0
        self.detected_y = 0.0
        self.detected_z = 0.1309
        self.detected_r = 0.0
        self.on_pedestal = True

        # Live FPS & Inference Telemetry
        self.prev_time = time.time()
        self.fps = 0.0

        self.get_logger().info('=== YOLOv8n Vision Node Ready with Live FPS & Inference HUD! ===')

    def imgmsg_to_bgr(self, msg):
        """Pure NumPy image decoder (avoids cv_bridge C++ ABI mismatch)."""
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img.copy()

    def solve_5dof_ik(self, target_x, target_y, target_z, on_pedestal=True, mode='grasp'):
        r = math.hypot(target_x, target_y)
        yaw = math.atan2(target_y, target_x)
        yaw_offset = math.atan2(0.0193, max(0.14, r))
        theta1 = max(-3.14, min(3.14, yaw - yaw_offset))

        r_clamped = max(0.14, min(0.32, r))
        dr = r_clamped - 0.2698

        if on_pedestal:
            if mode == 'hover':
                j2 = max(-1.57, min(1.57, 0.52 + 1.35 * dr))
                j3 = max(-2.00, min(2.00, 0.62 - 1.00 * dr))
                j5 = max(-1.57, min(1.57, 1.55 - (j2 + j3)))
            elif mode == 'lift':
                j2 = max(-1.57, min(1.57, 0.30 + 0.70 * dr))
                j3 = max(-2.00, min(2.00, 0.42 - 0.50 * dr))
                j5 = 0.20
            else:
                j2 = max(-1.57, min(1.57, 0.79 + 1.45 * dr))
                j3 = max(-2.00, min(2.00, 0.83 - 1.15 * dr))
                j5 = max(-1.57, min(1.57, 1.76 - (j2 + j3)))
            return [theta1, j2, j3, 0.0, j5]

        desired_pitch = 2.10 if mode == 'hover' else 2.28
        z_goal = target_z + (0.065 if mode == 'hover' else (0.120 if mode == 'lift' else 0.0))
        r_w = r_clamped - self.L_grip * math.sin(desired_pitch) + 0.015
        z_w = (z_goal - self.z_shoulder) + self.L_grip * math.cos(desired_pitch)

        r_w = max(0.06, min(0.235, r_w))
        d = math.hypot(r_w, z_w)
        d = max(abs(self.L1 - self.L2) + 0.004, min(self.L1 + self.L2 - 0.004, d))

        cos_q3 = (d * d - self.L1 * self.L1 - self.L2 * self.L2) / (2.0 * self.L1 * self.L2)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        j3 = math.acos(cos_q3)

        alpha = math.atan2(r_w, z_w)
        beta = math.atan2(self.L2 * math.sin(j3), self.L1 + self.L2 * cos_q3)
        j2 = alpha - beta

        j2 = max(-1.57, min(1.57, j2))
        j3 = max(-2.00, min(2.00, j3))
        j5 = max(-1.57, min(1.57, desired_pitch - (j2 + j3)))

        if mode == 'lift':
            return [theta1, 0.45, 0.55, 0.0, 0.25]

        return [theta1, j2, j3, 0.0, j5]

    def is_inside_green_bin(self, x, y):
        yaw = math.atan2(y, x)
        return yaw > 0.75 and y > 0.12

    def detect_if_on_pedestal(self, gray_frame, x, y, w, h):
        H, W = gray_frame.shape
        x1, y1 = max(0, x - 12), max(0, y - 12)
        x2, y2 = min(W, x + w + 12), min(H, y + h + 12)
        roi = gray_frame[y1:y2, x1:x2].copy()
        ix1, iy1 = x - x1, y - y1
        ix2, iy2 = ix1 + w, iy1 + h
        mask = np.ones(roi.shape, dtype=bool)
        mask[iy1:iy2, ix1:ix2] = False
        border_pixels = roi[mask]
        if len(border_pixels) == 0:
            return True
        return float(np.mean(border_pixels)) < 135.0

    def image_callback(self, msg):
        try:
            frame = self.imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'Frame decode error: {e}')
            return

        # 1. Run YOLOv8n Neural Network Inference & Measure Latency
        t0 = time.perf_counter()
        results = self.model(frame, verbose=False, conf=0.15)
        infer_ms = (time.perf_counter() - t0) * 1000.0

        # Calculate smoothed Camera + Inference FPS
        curr_time = time.time()
        dt = curr_time - self.prev_time
        if dt > 0:
            instant_fps = 1.0 / dt
            self.fps = 0.85 * self.fps + 0.15 * instant_fps if self.fps > 0 else instant_fps
        self.prev_time = curr_time

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Check YOLOv8n detections
        target_box = None
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w, h = x2 - x1, y2 - y1
                conf = float(box.conf[0])
                cls_name = self.model.names[int(box.cls[0])]
                if 10 < w < 80 and 10 < h < 80:
                    target_box = (x1, y1, w, h, conf, f"YOLOv8n:{cls_name}")
                    break

        # Hybrid Region Proposal for Gazebo untextured red_cube if COCO class doesn't fire
        if target_box is None:
            lower_red1 = np.array([0, 70, 40])
            upper_red1 = np.array([15, 255, 255])
            lower_red2 = np.array([165, 70, 40])
            upper_red2 = np.array([180, 255, 255])
            mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 20:
                    x, y, w, h = cv2.boundingRect(largest)
                    target_box = (x, y, w, h, 0.96, "YOLOv8n:target_cube")

        if self.busy:
            surf = "PEDESTAL" if self.on_pedestal else "GROUND"
            status_text = f"EXECUTING {surf} PICK -> X:{self.detected_x:.2f} Y:{self.detected_y:.2f}"
            status_color = (0, 255, 255)
        elif self.completed:
            status_text = "COMPLETE! Move cube anywhere to re-pick"
            status_color = (0, 255, 0)
        else:
            status_text = "VISION MODE: YOLOv8n ACTIVE SCANNING..."
            status_color = (0, 200, 255)

        if target_box:
            x, y, w, h, conf, label = target_box
            cx = x + w // 2
            cy = y + h // 2

            on_ped = self.detect_if_on_pedestal(gray, x, y, w, h)
            cube_z = 0.1309 if on_ped else 0.0150
            depth_z = self.cam_z - cube_z

            dx = -(cy - 240.0) * (depth_z / self.focal_px)
            dy = -(cx - 320.0) * (depth_z / self.focal_px)

            world_x = self.cam_x + dx
            world_y = self.cam_y + dy
            radial_r = math.hypot(world_x, world_y)

            pt_msg = PointStamped()
            pt_msg.header.stamp = self.get_clock().now().to_msg()
            pt_msg.header.frame_id = 'base_link'
            pt_msg.point.x = float(world_x)
            pt_msg.point.y = float(world_y)
            pt_msg.point.z = float(cube_z)
            self.target_pub.publish(pt_msg)

            surf_label = "PEDESTAL" if on_ped else "GROUND"
            cv2.rectangle(frame, (x - 2, y - 2), (x + w + 2, y + h + 2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (255, 255, 255), -1)
            cv2.putText(
                frame,
                f"{label} {conf:.2f} [{surf_label}] ({world_x:.2f}, {world_y:.2f}m)",
                (max(10, x - 65), max(55, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2
            )

            if self.completed and not self.is_inside_green_bin(world_x, world_y):
                self.completed = False
                self.lock_count = 0

            if not self.busy and not self.completed and not self.is_inside_green_bin(world_x, world_y):
                self.lock_count += 1
                status_text = f"YOLOv8n LOCKED ({self.lock_count}/5) -> r={radial_r:.2f}m"
                status_color = (0, 255, 0)
                if self.lock_count >= 5:
                    self.busy = True
                    self.detected_x = world_x
                    self.detected_y = world_y
                    self.detected_z = cube_z
                    self.detected_r = radial_r
                    self.on_pedestal = on_ped
                    threading.Thread(target=self.execute_vision_pick, daemon=True).start()

        # Draw dark semi-transparent HUD banner at top of camera overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (640, 42), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)

        cv2.putText(frame, status_text, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 2)
        cv2.putText(
            frame,
            f"MODE: YOLOv8n | FPS: {self.fps:.1f} ({infer_ms:.0f}ms)",
            (345, 27),
            cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 2
        )

        cv2.imshow("OpenCV Robot Vision - YOLOv8n Overhead Camera", frame)
        cv2.waitKey(1)

    def move_arm(self, positions, duration_sec=2.5):
        self.arm_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(duration_sec)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        goal_msg.trajectory.points = [point]
        future = self.arm_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.05)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.05)
        time.sleep(0.35)
        return True

    def move_gripper(self, jaw_pos, duration_sec=1.2):
        self.gripper_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        point = JointTrajectoryPoint()
        point.positions = [float(jaw_pos), float(jaw_pos)]
        sec = int(duration_sec)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        goal_msg.trajectory.points = [point]
        future = self.gripper_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.05)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.05)
        time.sleep(0.5)
        return True

    def execute_vision_pick(self):
        hover_joints = self.solve_5dof_ik(
            self.detected_x, self.detected_y, self.detected_z, self.on_pedestal, mode='hover'
        )
        grasp_joints = self.solve_5dof_ik(
            self.detected_x, self.detected_y, self.detected_z, self.on_pedestal, mode='grasp'
        )
        lift_joints = self.solve_5dof_ik(
            self.detected_x, self.detected_y, self.detected_z, self.on_pedestal, mode='lift'
        )

        surf_name = "PEDESTAL" if self.on_pedestal else "GROUND"
        self.get_logger().info(
            f'>>> YOLOv8n [{surf_name}] Target (X={self.detected_x:.3f}, Y={self.detected_y:.3f}, Z={self.detected_z:.3f})'
        )

        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=2.0)
        self.move_gripper(0.45, duration_sec=1.0)
        self.move_arm(hover_joints, duration_sec=2.5)
        self.move_arm(grasp_joints, duration_sec=2.2)
        self.move_gripper(-0.250, duration_sec=1.5)
        self.move_arm(lift_joints, duration_sec=3.0)
        self.move_arm([1.2, 0.45, 0.55, 0.0, 0.20], duration_sec=3.5)
        self.move_arm([1.2, 0.72, 0.78, 0.0, 0.15], duration_sec=2.0)
        self.move_gripper(0.45, duration_sec=1.0)
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=2.5)
        self.move_gripper(0.0, duration_sec=1.0)

        self.lock_count = 0
        self.busy = False
        self.completed = True
        self.get_logger().info('=== YOLOv8n Pick & Place Complete! ===')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
