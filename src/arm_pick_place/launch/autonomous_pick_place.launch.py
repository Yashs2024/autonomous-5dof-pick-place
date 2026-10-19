import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_arm_desc = get_package_share_directory('arm_description')

    # 1. Launch Gazebo Harmonic World + Robot + Pedestal + Green Bin + Camera + Controllers
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm_desc, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. Launch 3D Dynamic IK Vision Pick-and-Place Node (after Gazebo boots)
    vision_node = Node(
        package='arm_pick_place',
        executable='yolov8_pick_place_node',
        name='yolov8_pick_and_place_node',
        output='screen'
    )

    # 3. Launch Hardware Serial Servo Bridge (streams <S1,S2,S3,S4,S5,S6> PWM angles)
    serial_bridge_node = Node(
        package='arm_pick_place',
        executable='servo_serial_bridge',
        name='servo_serial_bridge',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(
            period=6.0,
            actions=[vision_node, serial_bridge_node]
        ),
    ])
