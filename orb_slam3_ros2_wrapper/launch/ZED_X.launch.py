import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Set environment variables
    robot_namespace = "ZED_X"
    robot_x = "1.0"
    robot_y = "1.0"

    # Set the config file path to the camera params
    config_file_path = "/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/ZED_X.yaml"

    # Set the log level
    log_level = "INFO"  # You can change this to INFO, WARN, ERROR, etc.

    orb_slam3_launch_file_dir = os.path.join(
        get_package_share_directory('orb_slam3_ros2_wrapper'), 'launch')

    orb_slam3_launch_file_path = os.path.join(
        orb_slam3_launch_file_dir, 'rgbd.launch.py')

    # Declare the config_file_path and log_level arguments to pass to rgbd.launch.py
    declare_config_file_path_arg = DeclareLaunchArgument(
        'config_file_path',
        default_value=config_file_path,
        description='Path to the configuration file for the ORB_SLAM3 node'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=log_level,
        description='Logging level for the ORB_SLAM3 node'
    )

    # Launch the rgbd.launch.py file
    orb_slam3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orb_slam3_launch_file_path),
        launch_arguments={
            'config_file_path': config_file_path,
            'log_level': log_level  # Pass the log_level argument
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='ROBOT_NAMESPACE', value=robot_namespace),
        SetEnvironmentVariable(name='ROBOT_X', value=robot_x),
        SetEnvironmentVariable(name='ROBOT_Y', value=robot_y),
        declare_config_file_path_arg,  # Declare the launch argument for config file path
        declare_log_level_arg,         # Declare the launch argument for log level
        orb_slam3_launch_description
    ])


if __name__ == '__main__':
    generate_launch_description()
