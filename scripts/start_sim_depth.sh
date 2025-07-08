#!/bin/bash
export PX4_UXRCE_DDS_NS=chotto
cd /root/PX4-Autopilot
make px4_sitl gz_x500_depth
