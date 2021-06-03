#!/usr/bin/env bash

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "16.04" ]; then
  ROS_NAME="kinetic"
elif [ $ubuntu_version == "18.04" ]; then
  ROS_NAME="melodic"
elif [ $ubuntu_version == "20.04" ]; then
  ROS_NAME="noetic"
else
  echo -e "Unsupported Ubuntu verison: $ubuntu_version"
  echo -e "Interbotix Arm only works with 16.04, 18.04, or 20.04"
  exit 1
fi

read -p "Install the Perception Pipeline (includes RealSense and AprilTag packages)? " resp
if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
  install_perception=true
else
  install_perception=false
fi

echo "Ubuntu $ubuntu_version detected. ROS-$ROS_NAME chosen for installation.";

echo -e "\e[1;33m ******************************************** \e[0m"
echo -e "\e[1;33m The installation may take around 15 Minutes! \e[0m"
echo -e "\e[1;33m ******************************************** \e[0m"
sleep 4
start_time="$(date -u +%s)"

# Update the system
sudo apt update && sudo apt -y upgrade
sudo apt -y autoremove

# Install some necessary core packages
sudo apt -y install openssh-server curl
if [ $ROS_NAME != "noetic" ]; then
  sudo apt -y install python-pip
  sudo -H pip install modern_robotics
else
  sudo apt -y install python3-pip
  sudo -H pip3 install modern_robotics
fi

# Step 1: Install ROS
if [ $(dpkg-query -W -f='${Status}' ros-$ROS_NAME-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing ROS..."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt update
  sudo apt -y install ros-$ROS_NAME-desktop-full
  if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
  fi
  echo "source /opt/ros/$ROS_NAME/setup.bash" >> ~/.bashrc
  if [ $ROS_NAME != "noetic" ]; then
    sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  else
    sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  fi
  sudo rosdep init
  rosdep update
else
  echo "ros-$ROS_NAME-desktop-full is already installed!"
fi
source /opt/ros/$ROS_NAME/setup.bash


if [ "$install_perception" = true ]; then
  # Step 2: Install Realsense packages

  # Step 2A: Install librealsense2
  if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo "Installing librealsense2..."
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main" -u
    if [ $ubuntu_version == "16.04" ]; then
      version="2.40.0-0~realsense0.3813"
    elif [ $ubuntu_version == "18.04" ]; then
      version="2.40.0-0~realsense0.3814"
    elif [ $ubuntu_version == "20.04" ]; then
      version="2.40.0-0~realsense0.3815"
    fi

    sudo apt -y install librealsense2-udev-rules=${version}
    sudo apt -y install librealsense2-dkms
    sudo apt -y install librealsense2=${version}
    sudo apt -y install librealsense2-gl=${version}
    sudo apt -y install librealsense2-gl-dev=${version}
    sudo apt -y install librealsense2-gl-dbg=${version}
    sudo apt -y install librealsense2-net=${version}
    sudo apt -y install librealsense2-net-dev=${version}
    sudo apt -y install librealsense2-net-dbg=${version}
    sudo apt -y install librealsense2-utils=${version}
    sudo apt -y install librealsense2-dev=${version}
    sudo apt -y install librealsense2-dbg=${version}
    sudo apt-mark hold librealsense2*
    sudo apt -y install ros-$ROS_NAME-ddynamic-reconfigure
  else
    echo "librealsense2 already installed!"
  fi

  # Step 2B: Install realsense2 ROS Wrapper
  REALSENSE_WS=~/realsense_ws
  if [ ! -d "$REALSENSE_WS/src" ]; then
    echo "Installing RealSense ROS Wrapper..."
    mkdir -p $REALSENSE_WS/src
    cd $REALSENSE_WS/src
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd realsense-ros/
    git checkout 2.2.20
    cd $REALSENSE_WS
    catkin_make clean
    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    catkin_make install
    echo "source $REALSENSE_WS/devel/setup.bash" >> ~/.bashrc
  else
    echo "RealSense ROS Wrapper already installed!"
  fi
  source $REALSENSE_WS/devel/setup.bash

  # Step 3: Install apriltag ROS Wrapper
  APRILTAG_WS=~/apriltag_ws
  if [ ! -d "$APRILTAG_WS/src" ]; then
    echo "Installing Apriltag ROS Wrapper..."
    mkdir -p $APRILTAG_WS/src
    cd $APRILTAG_WS/src
    git clone https://github.com/AprilRobotics/apriltag.git
    git clone https://github.com/AprilRobotics/apriltag_ros.git
    cd $APRILTAG_WS
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make_isolated
    echo "source $APRILTAG_WS/devel_isolated/setup.bash" >> ~/.bashrc
  else
    echo "Apriltag ROS Wrapper already installed!"
  fi
  source $APRILTAG_WS/devel_isolated/setup.bash

fi

# Step 4: Install Arm packages
INTERBOTIX_WS=~/interbotix_ws
if [ ! -d "$INTERBOTIX_WS/src" ]; then
  echo "Installing ROS packages for the Interbotix Arm..."
  mkdir -p $INTERBOTIX_WS/src
  cd $INTERBOTIX_WS/src
  git clone https://github.com/Interbotix/interbotix_ros_core.git
  git clone https://github.com/Interbotix/interbotix_ros_manipulators.git
  git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
  cd interbotix_ros_manipulators && git checkout $ROS_NAME && cd ..
  rm interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
  rm interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE
  if [ "$install_perception" = true ]; then
    rm interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/CATKIN_IGNORE
    rm interbotix_ros_toolboxes/interbotix_perception_toolbox/CATKIN_IGNORE
  fi
  rm interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
  rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE
  cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
  sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  cd $INTERBOTIX_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  echo "source $INTERBOTIX_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix Arm ROS packages already installed!"
fi
source $INTERBOTIX_WS/devel/setup.bash

# Step 5: Setup Environment Variables
if [ -z "$ROS_IP" ]; then
  echo "Setting up Environment Variables..."
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to reboot the computer before using the robot!"
