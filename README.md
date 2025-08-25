# PX4 SITL ROS 2
This repository contains all the necessary parts to build and run a docker image with SITL simulation on ROS2 leveraging XRCE to communicate with PX4.

## build image

`cd docker_ws` <br>
`./build_SITL_ros2.sh`


## Run container
on the root of the repository folder run:<br>
`./run_SITL_ros2.sh` this run scripts may need additional volumes to be mapped in the long run.


## Run simulation
Inside the container run:<br>
`./scripts/start_sim_depth.sh` <br>
In another terminal inside the container opened using **exec_SITL_ros2.sh** run XRCE:<br>
`./scripts/start_dds.sh` <br>

## Run bridges (camera, depth camera, point cloud, tfs)
Inside the container go to ws_bridge folder, build with colcon build, source install/setup.bash and then run: <br>
`ros2 launch gz_bridge_utilities custom_bridge.launch.py` <br>

## Run mapping (OctoMap), navigation (Nav2) and obstacle avoidance
In separate terminals inside the container (after sourcing your workspace):

**OctoMap server**<br>
`ros2 launch offboard_control octomap_server.launch.py` <br>

**Nav2 bringup** (sim time, autostart, no SLAM, no external localization):<br>
`ros2 launch nav2_bringup navigation_launch.py \
     use_sim_time:=true autostart:=true \
     params_file:=/root/scripts/nav2_drone_params.yaml \
     slam:=false localization:=false` <br>

**Obstacle avoidance node**<br>
`ros2 run offboard_control obstacle_avoidance` <br>

## result
These steps will give you a running simulation with px4 and X500v2 + depth camera + point clouds bridged on ros2, with OctoMap and Nav2 launched for planning, and the obstacle_avoidance node controlling the vehicle.
