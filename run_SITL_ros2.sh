#!/bin/bash
xhost +
docker run --rm -it --privileged --ipc host \
    --net host \
    -v ./scripts:/root/scripts \
    -v ./bridge_ws:/root/bridge_ws \
    -v ./ros2_offboard_ws:/root/ros2_offboard \
    -v ./ros2_px4_ws:/root/ros2_px4_ws \
    -v ./PX4-sim-patches/r1_rover:/root/PX4-Autopilot/Tools/simulation/gz/models/r1_rover/ \
    -v ./PX4-sim-patches/x500_model.sdf:/root/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf \
    -v ./PX4-sim-patches/x500_depth_model.sdf:/root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf \
    -v ./PX4-sim-patches/default_world_arena.sdf:/root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf \
    -v ./fastDDS_config:/root/fastDDS_config \
	-v /dev:/dev \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v ~/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -w /root \
    --name custom_ros2_sitlv1.14 \
    custom_sitl_ros2:v1.14 bash 

