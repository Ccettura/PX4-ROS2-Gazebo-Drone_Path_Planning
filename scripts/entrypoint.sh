#!/bin/bash
set -e

# Base ROS (serve per buildare)
source /opt/ros/humble/setup.bash

maybe_build () {
  local WS="$1"
  local DIR="$HOME/$WS"
  [ -d "$DIR/src" ] || return 0
  if [ ! -f "$DIR/install/setup.bash" ]; then
    echo "[entrypoint] Building $WS ..."
    cd "$DIR"
    colcon build --symlink-install
  fi
}

# Se esistono, builda solo se manca install/
maybe_build ws_sensor_combined
#maybe_build octomap_ws
maybe_build bridge_ws
maybe_build ros2_offboard

# A questo punto .bashrc conterrà i vari "source ..." (vedi Dockerfile sotto)
# Lo sorgiamo esplicitamente così vale anche per shell non interattive.
[ -f "$HOME/.bashrc" ] && source "$HOME/.bashrc"

exec "$@"
