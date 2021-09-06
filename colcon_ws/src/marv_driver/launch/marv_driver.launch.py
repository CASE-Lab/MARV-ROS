# Note will only work on foxy, will need to be slightly rewritten to launch using eloquent
# See: https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/

# Will make stdout unaccesible, if required add namespace via ros-args (see readme)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('marv_driver'),
        'config',
        'scenarios.yaml'
        )

    logger = launch.substitutions.LaunchConfiguration("log_level")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='power_management',
            name='power_management',
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
	    launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='can_bridge',
            name='can_bridge',
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='scenario_handler',
            name='scenario_handler',
            parameters = [config],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='logger',
            name='logger',
            parameters = [config],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='heartbeat_acu',
            name='heartbeat_acu',
            parameters = [config],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='status_sender',
            name='status_sender',
            parameters = [config],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='marv_driver',
            #namespace='',
            node_executable='sbg_interface',
            name='sbg_interface',
            parameters = [config],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
    ])
