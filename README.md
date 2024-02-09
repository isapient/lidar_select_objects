# Lidar_Select_Objects Package for ROS1 Noetic

## Overview

This ROS package filters obstacles from LiDAR point cloud frame by frame. This is the most robust solution: no complicated inter-frame model, no external data.

First, the ground points and buggy hull are detected and classified as not significant. Empty LiDAR pulses (0;0;0) are excluded from the LiDAR cloud.

Second, other objects are clusterized and classified by "penetrability" metrics. Objects that are big enough and more rigid than sparse are considered as obstacles. Others are dust cloud or noise LiDAR measurements.

The project still contains debug code parts, which should be removed during industrialization.

Algorithm description: Lidar_Select_Objects.pdf

## Future Improvements

- Second more accurate round of object clusterization based on primary clusters centroid
- Try to modify data collection to reach LiDAR second response; it can help in filtering dust.
- Use EKF for LiDAR IMU, accumulate short history in the world coordinate system to remove dust cloud.
- Fuse pictures from 3 lidars using full EKF localization with GPS, create Lidar SLAM

## Installation

1. Clone this repository into your ROS workspace's src directory:
   git clone https://github.com/isapient/lidar_select_objects.git

2. Build the package:\
   cd /path/to/your/ros/workspace
   catkin_make

3. Launch the node:\
   roslaunch lidar_select_objects lidar_select_objects.launch

4. Replay source data from bagfile:\
   rosbag play ~/LiDARFilteringAssignment.bag

5. Check output topics `/lidar_filter/signal` and `/lidar_filter/noise`\
I use rqt_bag for recording bags and Foxglove Studio for visualizing both bag files and live streams.

## Parameters

- `input_topic`: ROS1 topic with source point cloud data (default `/mbuggy/os3/points`)

## Docker

1. Build Docker image:\
docker build -t lidar_filter .

2. Run container with mounted bag:\
docker run -it --rm --name lidar_container -v ~/LiDARFilteringAssignment.bag:/ros/bagfile lidar_filter

3. Feed it by data:\
docker exec -it lidar_container /bin/bash -c "source /opt/ros/noetic/setup.bash && rosbag play /ros/bagfile"


## Contact
- Dmitry Popov
- Email: ppdmitry@gmail.com
