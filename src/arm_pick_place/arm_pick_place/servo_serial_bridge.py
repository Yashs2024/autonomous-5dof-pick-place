#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False


class ServoSerialBridge(Node):
    def __init__(self):
        super().__init__('servo_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.ser = None
        if HAS_PYSERIAL:
            for candidate in [port, '/dev/ttyUSB0', '/dev/ttyACM0']:
                try:
                    self.ser = serial.Serial(candidate, baud, timeout=0.05)
                    self.get_logger().info(f'Connected to physical microcontroller on {candidate} @ {baud} baud!')
                    break
                except Exception:
                    pass

        if self.ser is None:
            self.get_logger().info(
                'No microcontroller detected on USB -> Running in Virtual Servo Telemetry Mode (printing degrees).'
            )

        self.target_joints = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper_left_joint'
        ]
        self.latest_rad = {name: 0.0 for name in self.target_joints}
        self.last_sent_packet = ""

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10
        )
        # Transmit servo angles at 20 Hz
        self.timer = self.create_timer(0.05, self.send_servo_packet)

    def joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.latest_rad:
                self.latest_rad[name] = pos

    def rad_to_servo_deg(self, joint_name, rad):
        """
        Maps URDF joint radians to physical hobby servo degrees [0, 180]
        with 90 deg as the neutral upright home position.
        """
        if joint_name == 'gripper_left_joint':
            # Map [-0.250 (closed), 0.450 (open)] -> [20 deg (closed), 130 deg (open)]
            norm = (rad - (-0.250)) / (0.450 - (-0.250))
            deg = 20.0 + norm * (130.0 - 20.0)
        else:
            deg = 90.0 + math.degrees(rad)

        return int(max(0, min(180, round(deg))))

    def send_servo_packet(self):
        degs = [self.rad_to_servo_deg(j, self.latest_rad[j]) for j in self.target_joints]
        # Packet format: <S1,S2,S3,S4,S5,S6>\n
        packet = f"<{degs[0]},{degs[1]},{degs[2]},{degs[3]},{degs[4]},{degs[5]}>\n"

        if packet != self.last_sent_packet:
            self.last_sent_packet = packet
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(packet.encode('utf-8'))
                except Exception as e:
                    self.get_logger().warn(f'Serial write error: {e}')
            else:
                self.get_logger().info(f'Servo PWM Packet -> {packet.strip()}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.ser and node.ser.is_open:
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
