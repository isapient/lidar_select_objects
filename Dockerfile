FROM ros:noetic-perception

# Install additional dependencies if needed
RUN apt-get update && apt-get install -y\
    # Add your dependencies here \
    && rm -rf /var/lib/apt/lists/*

# Set up your workspace and copy your ROS package
WORKDIR /catkin_ws/src
COPY . /catkin_ws/src/lidar_select_objects

# Build your ROS package
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source your ROS workspace setup.bash in the entrypoint and launch your ROS package
ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && echo 'ROS workspace sourced' && roslaunch lidar_select_objects lidar_select_objects.launch"]

# Set the ROS_PACKAGE_PATH
ENV ROS_PACKAGE_PATH=/catkin_ws/src:$ROS_PACKAGE_PATH
