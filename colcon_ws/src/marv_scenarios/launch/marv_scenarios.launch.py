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
    config_params = os.path.join(
        get_package_share_directory('waverunner_scenarios'),
        'config',
        'parameters.yaml'
        )

    logger = launch.substitutions.LaunchConfiguration("log_level")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='example_scenario',
            name='example_scenario',
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
	    launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='test_throttle',
            name='test_throttle',
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='test_constant_velocity',
            name='test_constant_velocity',
            parameters = [config_params],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='set_reference_position',
            name='set_reference_position',
            parameters = [config_params],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
	    launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='test_throttle_ramp',
            name='test_throttle_ramp',
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),	
        launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='waypoint_navigation',
            name='waypoint_navigation',
            parameters = [config_params],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='thrust_controller_test',
            name='thrust_controller_test',
            parameters = [config_params],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='waverunner_scenarios',
            #namespace='',
            node_executable='heading_controller_test',
            name='heading_controller_test',
            parameters = [config_params],
            output='screen', 
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        ),
    ])
