.PHONY: all simulation dds gazebo_bridge

# SHELL := /bin/bash
# .SHELLFLAGS := -ec

all: simulation dds gazebo_bridge

simulation:
	@echo "Launching simulation..."
	bash ./start_sim_depth.sh > /dev/null &

dds:
	@echo "Starting DDS agent..."
	bash ./start_dds.sh &

gazebo_bridge:
	@echo "Starting ROS2Gazebo bridge..."
	bash ./start_ros2gz_bridge.sh &

stop:
	@echo "Stopping all background processes..."
	-pkill -9 -f px4
	-pkill -9 -f ruby
	-pkill -9 -f MicroXRCEAgent
	-pkill -9 -f image_bridge
	-pkill -9 -f param_bridg
	-pkill -9 -f static_transfor