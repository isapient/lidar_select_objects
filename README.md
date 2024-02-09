Lidar_Select_Objects Package for ROS1 noetic

OVERVIEW

This ROS package filters obstacles from LiDAR point cloud frame by frame.
This is the most robust solution: no complicated inter-frame model, no external data.

First, the ground points and buggy hull are detected and classified as not-significant.
Empty lidar pulses (0;0;0) is excluded from the lidar cloud.

Second, other objects clusterized and classified by "penetrability" metrics.
Objects that are big enough and more rigid than sparce considered as obstacles.
Others are dust cloud or noise lidar measuremen.

FUTURE IMPROVEMENTS

- Use EKF for Lidar IMU, accumulate short history in world coordinate system to remove dust cloud.
- Try to modify data collection to reach lidar second response, it can help filtering dust.

INSTALLATION

1. Clone this repository into your ROS workspace's src directory:
   git clone https://github.com/isapient/lidar_select_objects.git

2. Build the package:
   cd /path/to/your/ros/workspace
   catkin_make

3. Launch the node:
   roslaunch lidar_select_objects lidar_select_objects.launch

4. Replay source data from bagfile
   rosbag play ~/LiDARFilteringAssignment.bag

5. Check output topics  /lidar_filter/signal and /lidar_filter/noise
   I use Foxglove Studio both for vizualizing bag files and live streams

DOCKER

Bild docker image
docker build -t lidar_filter .

Run container with mounted bag
docker run -it --rm --name lidar_container     -v ~/LiDARFilteringAssignment.bag:/ros/bagfile     lidar_filter

Feed it by data
docker exec -it lidar_container /bin/bash -c "source /opt/ros/noetic/setup.bash && rosbag play /ros/bagfile"


PARAMETERS

input_topic: ROS1 topic with source point cloud data


Contact: Dmitry Popov
Email: ppdmitry@gmail.com
