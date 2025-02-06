from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction , IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get package share directory
    package_name = 'diffdrive_description'
    pkg_share = FindPackageShare(package_name).find(package_name)
    lidar_share=FindPackageShare('ydlidar_ros2_driver').find('ydlidar_ros2_driver')
    
    # Paths to config files
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    controller_params_path = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    
    # Get URDF via xacro
    robot_description_content = Command(['xacro ', urdf_path])
    
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': False}
    
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )
    robot_description1 = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description1},controller_params_path]
        # output='screen'
    )

    # Make sure controllers are loaded after controller_manager is ready
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager_node])

    # Spawn Controllers after controller_manager is ready
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[TimerAction(period=5.0,actions=[diff_drive_spawner])],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Delay diff_drive_spawner until controller_manager is ready
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[TimerAction(period=5.0,actions=[joint_broad_spawner])],
        )
    )

    # Delay joint_broad_spawner until controller_manager is ready

    # Add EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf_new.yaml')]
    )

    # YDLidar node
    ydlidar_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    lidar_share,'launch','ydlidar_launch.py'
                )])
    )
    imu_script_path = os.path.join(pkg_share, 'scripts', 'imu.py')
    imu_node = ExecuteProcess(
        cmd=['python3', imu_script_path],
        name='imu_publisher',
        output='screen'
    )
    velocity_script_path = os.path.join(pkg_share, 'scripts', 'velocity_forwarder.py')
    velocity_node = ExecuteProcess(
        cmd=['python3', velocity_script_path],
        name='velovity_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_pub_node,
        joint_state_pub_node,
        imu_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        ekf_node,
        ydlidar_node,
        velocity_node
    ])