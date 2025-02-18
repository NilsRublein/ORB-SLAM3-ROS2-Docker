# Fork Details

This fork contains so far the following changes:
* Added parameter to (not) use inertial mode
* Added comments to 
  * `type_conversions.cpp`
  * `orb_slam3_interface.cpp`
  * `rgbd-slam-node.cpp`
* Added `octomap_server` to the main launch file
* Added scripts for testing on [TUM rgbd dataset](https://cvg.cit.tum.de/rgbd/dataset/)
* Added simple point cloud accumulation for dense mapping (in [ORBSLAM3](https://github.com/NilsRublein/ORB_SLAM3), not the wrapper)
* Added publishers for publishing
  * The loop closure corrected pose
  * The pose before loop closure detection
  * Loop closure detection

## TODO
### Publishing Odometry 

* Determine how to calculate co-variances for sensor fusion. 
* Verify that uncorrected pose is indeed without loop closure corrections. A publisher has been already added that indicates loop closure detection. → Compare corrected and uncorrected pose.

### Dense Mapping

* ROS2 publishing

  * Port the dense map from ORBSLAM3 into the ROS2 wrapper, convert from OpenCV frame to ROS frame and publish as `PointCloud2` message.
  * Verify that the this map works as expected with the Octomap server (works already with sparse map).

* ZED Depth estimation

  * Currently we are using the ZED-ROS2-Wrapper for depth estimation which significantly reduces performances of the normal camera images. Either invest time into fixing performance issues of ZED-ROS2-Wrapper or look into alternatives for robust depth estimation methods (e.g. deep stereo matching networks). 

  * Add a filter to reduce noise in the depth estimation

### Other

* General Clean up & merge with latest updates from forked repo.
* Add a feature that publishes the number of tracked features by ORBSLAM3.
* Add some parametization to the dense pointcloud creation (e.g. resolution)
   
## TUM1 Rosbags & launchfile

Download the TUM1 rosbag from the [TUM rgbd dataset](https://cvg.cit.tum.de/rgbd/dataset/) and convert it to a ROS2 bag:

```bash
# If required, install rosbags:
pip install rosbags

# Convert ROS1 bag to a ROS2 bag
rosbags-convert --src rgbd_dataset_freiburg1_xyz.bag --dst ./rgbd_dataset_freiburg1_xyz_ROS2_bag
```

Run the ROS bag:
```bash
# This docker container expects ROS_DOMAIN_ID=55. If not already set, you can run:
export ROS_DOMAIN_ID=55

# Start ROS bag in one terminal with renamed topics 
ros2 bag play rgbd_dataset_freiburg1_xyz_ROS2_bag/ --remap /camera/rgb/image_color:=robot_0/rgb_camera /camera/depth/image:=robot_0/depth_camera
```

Run ORB-SLAM3:
```bash
# Launch docker container and node in a another terminal
sudo docker compose run orb_slam3_22_humble
ros2 launch orb_slam3_ros2_wrapper TUM1.launch.py 
```

## ZED2 / ZEDX Rosbags and launchfile

Below you can find instructions run a rosbag from either ZED2 or ZEDX camera. 
Note that there have been different launch files created yfor different camera resolutions. As the camera resolution changes, also the intrinsic parameters of the calibration file changes. The different launch files therefore simply load different `camera.yaml` files.

**@TODO**: Simply link in the config file to the correct `camera.yaml` file instead of having different launch files. This way you only need to change parameters in one place.

Run the ROS bag:
```bash
# This docker container expects ROS_DOMAIN_ID=55. If not already set, you can run:
export ROS_DOMAIN_ID=55

# Start ROS bag in one terminal with renamed topics 
ros2 bag play outdoor_flight_4 --remap /zed/zed_node/left_raw_gray/image_raw_gray:=robot_0/rgb_camera /zed/zed_node/depth/depth_registered:=robot_0/depth_camera /zed/zed_node/imu/data_raw:=robot_0/imu
```

Run ORB-SLAM3:
```bash
# Launch docker container and node in a another terminal
sudo docker compose run orb_slam3_22_humble
ros2 launch orb_slam3_ros2_wrapper ZED_2_VGA.launch.py # Replace camera resolution if needed!
```

## Octomap in Rviz2

`rgbd.launch.py` has been modified to also launch an `octomap_server`. 

If you want to visualize the octomap in Rviz2 outside the docker, install the following dependencies on your machine via:
```bash
sudo apt install libpcl1 ros-humble-octomap-*
```

Because there are some [errors](https://robotics.stackexchange.com/questions/112732/error-loading-octomap-using-rviz2-plugin) for the octomap Rviz2 plugin, we have to start RVIZ as follows: 
```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 run rviz2 rviz2
```

Finally, in Rviz2, select from `octomap_rviz_plugins` the `OccupancyGrid` plugin to display the 3D octomap.

## Troubleshooting
* Make sure you have the same `ROS_DOMAIN_ID`
* If you see `Waiting for Image` in the ORBSLAM3 Pangolin viewer, make sure you
  * You are remapping the topics correctly. 
  * If you enabled IMU measurements, ORBSLAM will expect IMU data and not run without it.
* You can run rosbags also from outside the docker container. However, to see the msgs from the topics you can use the following DDS settings as a workaround: [Issue #8](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker/issues/8)

***

# ORB-SLAM3 ROS2 Wrapper Docker

This repository contains a dockerized comprehensive wrapper for ORB-SLAM3 on ROS 2 Humble for Ubuntu 22.04.

# Demo GIF

![ORBSLAM3-GIF](orbslam3.gif)

# Steps to use this wrapper

## 1. Clone this repository

1. ```git clone https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker```
2. ```cd ORB-SLAM3-ROS2-Docker```
3. ```git submodule update --init --recursive --remote```

## 2. Install Docker on your system

```bash
cd ORB-SLAM3-ROS2-Docker
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```

## 3. Build the image with ORB_SLAM3

1. Build the image: ```sudo docker build -t orb-slam3-humble:22.04 .```
2. Add ```xhost +``` to your ```.bashrc``` to support correct x11-forwarding using ```echo "xhost +" >> ~/.bashrc```
3. ```source ~/.bashrc```
4. You can see the built images on your machine by running ```sudo docker images```.

## 4. Running the container

1. ```cd ORB-SLAM3-ROS2-Docker``` (ignore if you are already in the folder)
2. ```sudo docker compose run orb_slam3_22_humble```
3. This should take you inside the container. Once you are inside, run the command ```xeyes``` and a pair of eyes should pop-up. If they do, x11 forwarding has correctly been setup on your computer.

## 5. Building the ORB-SLAM3 Wrapper

Launch the container using steps in (4).
```bash
cd /root/colcon_ws/
colcon build --symlink-install
source install/setup.bash
```

## Launching ORB-SLAM3

Launch the container using steps in (4).
If you are inside the container, run the following:

1. ```ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py```
3. You can adjust the initial co-ordinates of the robot along with its namespace in the ```unirobot.launch.py``` file.

## Running this with a Gazebo Classic simulation.

1. Setup the ORB-SLAM3 ROS2 Docker using the steps above. Once you do (1) step in the ```Launching ORB-SLAM3``` section, you should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
2. Setup the simulation by following the README [here](https://github.com/suchetanrs/gz-sim-environment)
3. Once you are able to teleop the robot, you should be able to run ORB-SLAM3 with both the containers (simulation and wrapper) running parallely.

### Potential issues you may face.
The simulation and the wrapper both have their ```ROS_DOMAIN_ID``` set to 55 so they are meant to work out of the box. However, you may face issues if this environment variable is not set properly. Before you start the wrapper, run ```ros2 topic list``` and make sure the topics namespaced with ```robot_0``` are visible inside the ORB-SLAM3 container provided the simulation is running along the side.


## Important notes

ORB-SLAM3 is launched from ```orb_slam3_docker_20_humble/orb_slam3_ros2_wrapper/launch/rgbd.launch.py``` which inturn is launched from ```orb_slam3_docker_20_humble/orb_slam3_ros2_wrapper/launch/unirobot.launch.py```

Currently the ```rgbd.launch.py``` launch file defaults to ```orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml```. You can modify this with your own parameter file in case you wish to use your own camera.

The very initial versions of this code were derived from [thien94/orb_slam3_ros_wrapper](https://github.com/thien94/orb_slam3_ros_wrapper) and [zang9/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)

## ROS Parameter descriptions
| Parameter Name          | Default Value | Description                                                                 |
|-------------------------|---------------|-----------------------------------------------------------------------------|
| `robot_base_frame`      | `base_footprint` | The name of the frame attached to the robot's base. |
| `global_frame`          | `map`         | The name of the global frame of reference. It represents a fixed world coordinate frame in which the robot navigates.|
| `odom_frame`            | `odom`        | The name of the odometry frame. |
| `robot_x`               | `0.0`         | The robot's initial x-coordinate in the global frame. Specifies the starting position along the x-axis. The SLAM Wrapper will assume this to be the initial x position|
| `robot_y`               | `0.0`         | The robot's initial y-coordinate in the global frame. Specifies the starting position along the y-axis. The SLAM Wrapper will assume this to be the initial y position|
| `visualization`         | `true`        | A boolean flag to enable or disable visualization. When set to `true`, the ORB-SLAM3 viewer will show up with the tracked points and the keyframe trajectories.|
| `ros_visualization`     | `false`       | A boolean flag to control ROS-based visualization. If set to `true`, it enables ROS tools like RViz to visualize the robot's data. (3D position of the tracked points etc.)  **This feature is unstable and not tested as of now**|
| `no_odometry_mode`      | `false`       | A boolean flag to toggle odometry mode. When `true`, the system operates without relying on odometry data, which might be used in scenarios where odometry information is unavailable or unreliable. In this case, it publishes the transform directly between the ```global_frame``` and the ```robot_base_frame```|