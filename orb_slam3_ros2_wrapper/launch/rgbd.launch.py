#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    #---------------------------------------------

    # Essential_paths
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')

    #---------------------------------------------

    # LAUNCH ARGS
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace =  LaunchConfiguration('robot_namespace')
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=TextSubstitution(text=ROBOT_NAMESPACE),
        description='The namespace of the robot')

    robot_x = LaunchConfiguration('robot_x')
    ROBOT_X = os.environ['ROBOT_X']
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value=TextSubstitution(text=ROBOT_X),
        description='The namespace of the robot')

    robot_y = LaunchConfiguration('robot_y')
    ROBOT_Y = os.environ['ROBOT_Y']
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value=TextSubstitution(text=ROBOT_Y),
        description='The namespace of the robot')

    config_file_path = LaunchConfiguration('config_file_path')
    declare_config_file_path_cmd = DeclareLaunchArgument(
        'config_file_path',
        default_value='/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/scout_v2_rgbd.yaml',
        description='Path to the configuration file')

    # Add a launch argument for log level
    log_level = LaunchConfiguration('log_level')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level for the ORB_SLAM3 node')

    #---------------------------------------------

    def all_nodes_launch(context):
        params_file = LaunchConfiguration('params_file')
        vocabulary_file_path = "/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(orb_wrapper_pkg, 'params', 'rgbd-ros-params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        param_substitutions = {
            'robot_base_frame': context.launch_configurations['robot_namespace'] + '/base_footprint',
            'odom_frame': context.launch_configurations['robot_namespace'] + '/odom',
            'robot_x': context.launch_configurations['robot_x'],
            'robot_y': context.launch_configurations['robot_y']
        }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=context.launch_configurations['robot_namespace'],
            param_rewrites=param_substitutions,
            convert_types=True)

        orb_slam3_node = Node(
            package='orb_slam3_ros2_wrapper',
            executable='rgbd',
            output='screen',
            namespace=context.launch_configurations['robot_namespace'],
            arguments=[
                vocabulary_file_path,
                context.launch_configurations['config_file_path'],
                '--ros-args',
                '--log-level',
                context.launch_configurations['log_level']  # Use log_level from LaunchConfiguration
            ],
            parameters=[configured_params])

        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch)
    #---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        robot_x_arg,
        robot_y_arg,
        declare_config_file_path_cmd,
        declare_log_level_cmd, 
        opaque_function
    ])
