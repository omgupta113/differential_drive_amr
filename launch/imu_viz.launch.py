from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Path to RViz configuration file
    rviz_config = PathJoinSubstitution([
        FindPackageShare('diffdrive_description'),
        'config',
        'imu_config.rviz'
    ])
    
    return LaunchDescription([
        # Start RViz2 with our configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # If you need coordinate frame transformation
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])