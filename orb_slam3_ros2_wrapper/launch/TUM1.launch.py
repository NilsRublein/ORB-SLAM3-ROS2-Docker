import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set environment variables
    robot_namespace = "robot_0"
    robot_x = "0.0"
    robot_y = "0.0"

    # Set the config file path to the camera params
    config_file_path = "/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/TUM1.yaml"

    # Set the log level
    log_level = "INFO"  # You can change this to INFO, WARN, ERROR, etc.

    orb_slam3_launch_file_dir = os.path.join(
        get_package_share_directory('orb_slam3_ros2_wrapper'), 'launch')

    orb_slam3_launch_file_path = os.path.join(
        orb_slam3_launch_file_dir, 'rgbd.launch.py')

    # Launch the rgbd.launch.py file
    orb_slam3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orb_slam3_launch_file_path),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "robot_x": robot_x,
            "robot_y": robot_y,
            'config_file_path': config_file_path,
            'log_level': log_level  # Pass the log_level argument
        }.items()
    )

    return LaunchDescription([
        orb_slam3_launch_description
    ])


if __name__ == '__main__':
    generate_launch_description()
