#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import xacro
import subprocess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#print("om")

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time',default='False')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('diffdrive_description'))
    param_file = os.path.join(pkg_path,'config','velocity.yaml')
    use_respawn=False

    use_composition = 'False'
    container_name_full='nav2_container'

    param_substitutions = {
        'use_sim_time': use_sim_time,
        }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    ld = LaunchDescription()


    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[configured_params]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],

            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['velocity_smoother']}]),
        #
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='gnome-terminal --'  # This will open the teleop node in a new terminal window
        )
        ]
    )
    

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[{param_file}]
            ),
            ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_localization',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': 'true',
                         'node_names': '/velocity_smoother'}]),
        ],
    )
    


    ld.add_action(load_nodes)
    #om=os.system('ros2 run nav2_util lifecycle_bringup velocity_smoother', shell=True)
    ld.add_action(load_composable_nodes)

    return ld