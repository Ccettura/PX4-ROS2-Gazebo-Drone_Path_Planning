#!/bin/bash

# Step 1: Lancia Gazebo in background
echo "[INFO] Starting Gazebo..."
gz sim /root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf &

# Step 2: Salva il PID per eventuali controlli futuri
GAZEBO_PID=$!

# Step 3: Attendi qualche secondo per far partire Gazebo
sleep 5

# Step 4: Naviga nella cartella PX4
cd /root/PX4-Autopilot || { echo "[ERROR] PX4 directory not found!"; exit 1; }

# Step 5: Imposta le variabili d'ambiente
echo "[INFO] Setting PX4 environment variables..."
source Tools/simulation/gz/setup_gz.sh
export PX4_UXRCE_DDS_NS=chotto
export PX4_GZ_MODEL=x500_depth
export PX4_GZ_MODEL_NAME=x500_depth_0
# export PX4_GZ_MODEL_POSE="3. 3. 0. 0. 0. 3.14159"  # opzionale

# Step 6: Avvia PX4
echo "[INFO] Starting PX4 simulation..."
make px4_sitl gz_x500_depth
