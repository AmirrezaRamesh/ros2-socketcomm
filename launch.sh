#!/bin/bash

# usage:
#     ./launch.sh <starting_number> <my_ip> <target_ip>
# example:
#     ./launch.sh 8000 192.168.1.10 192.168.1.20

STARTING_NUMBER="$1"
MY_IP="$2"
TARGET_IP="$3"
REMOTE_USER="$4"

if [ -z "$STARTING_NUMBER" ] || [ -z "$MY_IP" ] || [ -z "$TARGET_IP" ] || [ -z "$REMOTE_USER" ]; then
  echo "error: Missing arguments."
  echo "usage: ./launch.sh <starting_number> <my_ip> <target_ip> <remote_user>"
  exit 1
fi

echo "starting remote python node on $TARGET_IP with target_IP:=$MY_IP"
ssh "$REMOTE_USER@$TARGET_IP" << EOF
tmux new-session -d -s collatz_session "bash -c 'sleep 2s && source /opt/ros/humble/setup.bash && source ~/ros2-socketcomm/install/setup.bash && ros2 run collatz_python_server python_server --ros-args -p target_IP:=$MY_IP; exec bash'"
EOF


echo "launching cpp node and visualizer with starting_number:=$STARTING_NUMBER target_IP:=$TARGET_IP"
ros2 launch collatz_core cpp_and_vis_launch.py starting_number:=$STARTING_NUMBER target_IP:=$TARGET_IP
