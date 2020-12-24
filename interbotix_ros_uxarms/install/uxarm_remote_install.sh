#!/usr/bin/env bash

read -p "What is the hostname of the robot computer (type 'hostname' in a terminal to find it)? " HOSTNAME

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "16.04" ]; then
  ROS_NAME="kinetic"
elif [ $ubuntu_version == "18.04" ]; then
  ROS_NAME="melodic"
elif [ $ubuntu_version == "20.04" ]; then
  ROS_NAME="noetic"
else
  echo -e "Unsupported Ubuntu verison: $ubuntu_version"
  echo -e "Interbotix Remote Arm only works with 16.04, 18.04, or 20.04"
  exit 1
fi

# Step 1: Install Arm packages
INTERBOTIX_WS=~/interbotix_ws
if [ ! -d "$INTERBOTIX_WS/src" ]; then
  echo "Installing Simulation/Visualization ROS packages for the Interbotix Arms..."
  mkdir -p $INTERBOTIX_WS/src
  cd $INTERBOTIX_WS
  catkin_make
  cd src
  git clone https://github.com/Interbotix/interbotix_ros_manipulators.git
  cd interbotix_ros_manipulators && git checkout $ROS_NAME && cd ..
  rm interbotix_ros_manipulators/interbotix_ros_uxarms/CATKIN_IGNORE
  echo "source $INTERBOTIX_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix Arm ROS packages already installed!"
fi
source $INTERBOTIX_WS/devel/setup.bash

# Step 5: Setup Environment Variables
if [ -z "$ROS_IP" ]; then
  echo "Setting up Environment Variables..."
  echo "export ROS_MASTER_URI=http://$HOSTNAME.local:11311" >> ~/.bashrc
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi

ORANGE='\033[;33m'
NC='\033[0m'

echo "Remote Installation Complete! Close this terminal and open a new one to finish."
echo -e "NOTE: Remember to comment out the ${ORANGE}source $INTERBOTIX_WS/devel/setup.bash${NC} and ${ORANGE}export ROS_MASTER_URI=http://$HOSTNAME.local:11311${NC} lines from the ~/.bashrc file when done using the Arm! Then close out of your terminal and open a new one."
