#!/bin/bash

source /opt/ros/humble/setup.bash
source /root/bridge_ws/install/local_setup.bash
# ros2 run ros_gz_image image_bridge /topic1 /topic2 / topic3 ....

# ros2 run ros_gz_image image_bridge /camera /depth_camera

ros2 launch gz_bridge_utilities custom_bridge.launch.py