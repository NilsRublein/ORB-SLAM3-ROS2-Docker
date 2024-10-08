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
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value="robot",
        description='The namespace of the robot')

    robot_x = LaunchConfiguration('robot_x')
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value="1.0",
        description='The namespace of the robot')

    robot_y = LaunchConfiguration('robot_y')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value="1.0",
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

    def all_nodes_launch(context, robot_namespace, robot_x, robot_y, config_file_path, log_level):
        params_file = LaunchConfiguration('params_file')
        vocabulary_file_path = "/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(orb_wrapper_pkg, 'params', 'rgbd-ros-params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        param_substitutions = {
            'robot_base_frame': robot_namespace.perform(context) + '/base_footprint',
            'odom_frame': robot_namespace.perform(context) + '/odom',
            'robot_x': robot_x.perform(context),
            'robot_y': robot_y.perform(context),
            'config_file_path': config_file_path.perform(context),
            'log_level': log_level.perform(context)
            }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=robot_namespace.perform(context),
            param_rewrites=param_substitutions,
            convert_types=True)

        orb_slam3_node = Node(
            package='orb_slam3_ros2_wrapper',
            executable='rgbd',
            output='screen',
            namespace=robot_namespace.perform(context),
            arguments=[vocabulary_file_path, config_file_path],
            parameters=[configured_params])

        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch, args=[robot_namespace, robot_x, robot_y, config_file_path, log_level])
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
