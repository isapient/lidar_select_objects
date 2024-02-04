Lidar_Select_Objects ROS Package

OVERVIEW

This ROS package filters LiDAR point cloud data based on z-coordinate (height), separating points into objects and non-objects.

INSTALLATION

1. Clone this repository into your ROS workspace's src directory:

   git clone https://github.com/isapient/lidar_select_objects.git

2. Build the package:

   cd /path/to/your/ros/workspace
   catkin_make

3. Launch the node:

   roslaunch lidar_select_objects lidar_select_objects.launch

PARAMETERS

Adjust z-coordinate threshold in the launch file.

CONTACT

- Dmitry Popov
- Email: ppdmitry@gmail.com