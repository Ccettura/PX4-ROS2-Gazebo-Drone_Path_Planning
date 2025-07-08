#!/bin/bash
export PX4_UXRCE_DDS_NS=chotto
cd /root/PX4-Autopilot
# PX4_GZ_WORLD=<custom world in PX4-Autopilot/Tools/simulation/gz/worlds> make px4_sitl gz_x500_depth
# PX4_GZ_MODEL_POSE="3. 3. 0. 0. 0. 3.14159" 
# PX4_GZ_MODEL_POSE="3.,3.,0.,0.,0.,3.14159" 
make px4_sitl gz_x500_depth
