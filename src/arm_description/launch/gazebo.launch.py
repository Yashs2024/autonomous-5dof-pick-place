import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# Tell Gazebo where to find the arm_description package meshes
from ament_index_python.packages import get_package_share_directory
import os
ws_share_dir = os.path.dirname(get_package_share_directory('arm_description'))
if 'GZ_SIM_RESOURCE_PATH' in os.environ:
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + ws_share_dir
else:
    os.environ['GZ_SIM_RESOURCE_PATH'] = ws_share_dir


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'arm.urdf.xacro')
    sdf_file = os.path.join(pkg_share, 'urdf', 'target_cube.sdf')
    cam_sdf_file = os.path.join(pkg_share, 'urdf', 'overhead_camera.sdf')
    
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Gazebo Sim launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'five_dof_arm', '-z', '0.01'],
        output='screen'
    )

    # Spawn Target Cube on Pedestal (Pickup coordinates: X=0.18, Y=0.11, Z=0.115)
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_file, '-name', 'target_cube', '-x', '0.2691', '-y', '0.0193', '-z', '0.1309'],
        output='screen'
    )

    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', cam_sdf_file, '-name', 'overhead_camera'],
        output='screen'
    )

    # ROS-GZ Bridge for Clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    # Controller Spawners
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_group_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_cube,
        spawn_camera,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller
    ])
