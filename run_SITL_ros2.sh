#!/bin/bash
xhost +
docker run --rm -it --privileged --ipc host \
    --net host\
	--runtime nvidia --gpus all \
    -v ./scripts:/root/scripts \
    -v ./bridge_ws:/root/bridge_ws \
    -v ./ros2_offboard_ws:/root/ros2_offboard \
    -v ./PX4-sim-patches/x500_model.sdf:/root/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf \
	-v /dev:/dev \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v ~/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -w /root \
    --name ros2_sitlv1.14 \
    sitl_ros2:v1.14 bash 

