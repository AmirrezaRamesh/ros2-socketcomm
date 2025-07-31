#!/bin/bash

# usage:
#     ./build.sh <target_ip> <target_user>
# example:
#     ./build.sh 192.168.1.10 ros_user

TARGET_IP="$1"
REMOTE_USER="$2"

if [ -z "$TARGET_IP" ] || [ -z "$REMOTE_USER" ]; then
    echo "error: missing argument(s)."
    echo "usage: ./build.sh <target_ip> <remote_user>"
    exit 1
fi

LOCAL_DIR="$HOME/ros2-socketcomm"
REMOTE_DIR="~/ros2-socketcomm"

echo "creating folders on remote..."
ssh "$REMOTE_USER@$TARGET_IP" "mkdir -p ~/ros2-socketcomm/src"

echo "copying files to remote..."
scp -r ./src/collatz_core "$REMOTE_USER@$TARGET_IP:~/ros2-socketcomm/src/"
scp -r ./src/collatz_python_server "$REMOTE_USER@$TARGET_IP:~/ros2-socketcomm/src/"

echo "building packages locally..."
cd $LOCAL_DIR && source $HOME/.bashrc && sleep 1s && colcon build --packages-select collatz_core collatz_cpp_server

echo "building packages remotely..."
ssh "$REMOTE_USER@$TARGET_IP" "cd ~/ros2-socketcomm && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

echo "changing bashrc locally..."
echo "source opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $LOCAL_DIR/install/setup.bash" >> ~/.bashrc

echo "sourcing bashrc locally for the launch command"
source ~/.bashrc 

echo "changing bashrc remotly..."
ssh "$REMOTE_USER@$TARGET_IP" "echo 'source ~/ros2-socketcomm/install/setup.bash' >> ~/.bashrc"

echo "build complete on both machines."
