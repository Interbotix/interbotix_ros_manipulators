#!/usr/bin/env bash

# USAGE: ./xsarm_amd64_install.sh [-h][-d DISTRO][-p PATH][-n]
#
# Install the Interbotix X-Series Arms packages and their dependencies.

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

PROMPT="> "

ALL_VALID_DISTROS=('melodic' 'noetic' 'galactic')
ROS1_VALID_DISTROS=('melodic' 'noetic')
ROS2_VALID_DISTROS=('galactic')

BIONIC_VALID_DISTROS=('melodic')
FOCAL_VALID_DISTROS=('noetic' 'galactic')

NONINTERACTIVE=false
DISTRO_SET_FROM_CL=false
INSTALL_PATH=~/interbotix_ws

_usage="${BOLD}USAGE: ./xsarm_amd64_install.sh [-h][-d DISTRO][-p PATH][-n]${NORM}

Install the Interbotix X-Series Arms packages and their dependencies.

Options:

  -h              Display this help message and quit

  -d DISTRO       Install the DISTRO ROS distro compatible with your Ubuntu version. See
                  'https://github.com/Interbotix/.github/blob/main/SECURITY.md' for the list of
                  supported distributions. If not given, installs the ROS1 Distro compatible with
                  your Ubuntu version.

  -p PATH         Sets the absolute install location for the Interbotix workspace. If not specified,
                  the Interbotix workspace directory will default to '~/interbotix_ws'.

  -n              Install all packages and dependencies without prompting. This is useful if
                  you're running this script in a non-interactive terminal like when building a
                  Docker image.

Examples:

  ./xsarm_amd64_install.sh ${BOLD}-h${NORM}
    This will display this help message and quit.

  ./xsarm_amd64_install.sh
    This will install just the ROS1 distro compatible with your Ubuntu version. It will prompt you
    to ask if you want to install certain packages and dependencies.

  ./xsarm_amd64_install.sh ${BOLD}-d noetic${NORM}
    This will install ROS1 Noetic assuming that your Ubuntu version is compatible.

  ./xsarm_amd64_install.sh ${BOLD}-n${NORM}
    Skip prompts and install all packages and dependencies.

  ./xsarm_amd64_install.sh ${BOLD}-d galactic${NORM}
    Install ROS2 Galactic assuming that your Ubuntu version is compatible.

  ./xsarm_amd64_install.sh ${BOLD}-d galactic -n${NORM}
    Install ROS2 Galactic and all packages and dependencies.

  ./xsarm_amd64_install.sh -p ~/custom_ws
    Installs the Interbotix packages under the '~/custom_ws' path."

function help() {
  # print usage
  cat << EOF
$_usage
EOF
}

# https://stackoverflow.com/a/8574392/16179107
function contains_element () {
  # check if an element is in an array
  local e match="$1"
  shift
  for e; do [[ "$e" == "$match" ]] && return 0; done
  return 1
}

function failed() {
  # Log error and quit with a failed exit code
  echo -e "${ERR}[ERROR] $1${RRE}"
  echo -e "${ERR}[ERROR] Interbotix Installation Failed!${RRE}"
  exit 1
}

function validate_distro() {
  # check if chosen distro is valid and set ROS major version
  if contains_element $ROS_DISTRO_TO_INSTALL "${ALL_VALID_DISTROS[@]}"; then
    if contains_element $ROS_DISTRO_TO_INSTALL "${ROS1_VALID_DISTROS[@]}"; then
      # Supported ROS1 distros
      ROS_VERSION_TO_INSTALL=1
    elif contains_element $ROS_DISTRO_TO_INSTALL "${ROS2_VALID_DISTROS[@]}"; then
      # Supported ROS2 distros
      ROS_VERSION_TO_INSTALL=2
    else
      # For cases where it passes the first check but somehow fails the second check
      failed "Something went wrong."
    fi
    ROS_DISTRO_TO_INSTALL=$ROS_DISTRO_TO_INSTALL
    echo -e "${GRN}${BOLD}Chosen Version: ROS${ROS_VERSION_TO_INSTALL} $ROS_DISTRO_TO_INSTALL${NORM}${OFF}"
    return 0
  else
    failed "'$ROS_DISTRO_TO_INSTALL' is not a valid ROS Distribution. Choose one of: "${ALL_VALID_DISTROS[@]}""
  fi
}

function check_ubuntu_version() {
 # check if the chosen distribution is compatible with the Ubuntu version
  case $UBUNTU_VERSION in

    18.04 )
      if contains_element $ROS_DISTRO_TO_INSTALL "${BIONIC_VALID_DISTROS[@]}"; then
        PY_VERSION=2
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    20.04 )
      if contains_element $ROS_DISTRO_TO_INSTALL "${FOCAL_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    *)
      failed "Something went wrong."
      ;;

  esac
}

function install_essential_packages() {
  # Install necessary core packages
  sudo apt -y install openssh-server curl
  if [ $PY_VERSION == 2 ]; then
    sudo apt -y install python-pip
    python -m pip install modern_robotics
  elif [ $PY_VERSION == 3 ]; then
    sudo apt -y install python3-pip
    python3 -m pip install modern_robotics
  else
    failed "Something went wrong."
  fi
}

function install_ros1() {
  # Step 1: Install ROS
  if [ $(dpkg-query -W -f='${Status}' ros-$ROS_DISTRO_TO_INSTALL-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo -e "${GRN}Installing ROS1 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt -y install ros-$ROS_DISTRO_TO_INSTALL-desktop-full
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    fi
    echo "source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash" >> ~/.bashrc
    if [ $PY_VERSION == 2 ]; then
      sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    elif [ $PY_VERSION == 3 ]; then
      sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    fi
    sudo rosdep init
    rosdep update
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop-full is already installed!"
  fi
  source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash

  if [ "$INSTALL_PERCEPTION" = true ]; then
    # Step 2: Install Realsense packages

    # Step 2A: Install librealsense2
    if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
      echo -e "${GRN}Installing librealsense2...${OFF}"
      sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
      sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main" -u
      if [ $UBUNTU_VERSION == "18.04" ]; then
        version="2.48.0-0~realsense0.4975"
      elif [ $UBUNTU_VERSION == "20.04" ]; then
        version="2.48.0-0~realsense0.4976"
      fi

      sudo apt -y install                   \
        librealsense2-udev-rules=${version} \
        librealsense2-dkms                  \
        librealsense2=${version}            \
        librealsense2-gl=${version}         \
        librealsense2-gl-dev=${version}     \
        librealsense2-gl-dbg=${version}     \
        librealsense2-net=${version}        \
        librealsense2-net-dev=${version}    \
        librealsense2-net-dbg=${version}    \
        librealsense2-utils=${version}      \
        librealsense2-dev=${version}        \
        librealsense2-dbg=${version}
      sudo apt-mark hold librealsense2*
      sudo apt -y install ros-$ROS_DISTRO_TO_INSTALL-ddynamic-reconfigure
    else
      echo "librealsense2 already installed!"
    fi

    # Step 2B: Install realsense2 ROS Wrapper
    REALSENSE_WS=~/realsense_ws
    if [ ! -d "$REALSENSE_WS/src" ]; then
      echo -e "${GRN}Installing RealSense ROS Wrapper...${OFF}"
      mkdir -p $REALSENSE_WS/src
      cd $REALSENSE_WS/src
      git clone https://github.com/IntelRealSense/realsense-ros.git -b 2.3.1
      cd $REALSENSE_WS
      catkin_make clean
      catkin_make install && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
      if [ $? -eq 0 ]; then
        echo -e "${GRN}${BOLD}Realsense ROS Wrapper built successfully!${NORM}${OFF}"
      else
        failed "Failed to build Realsense ROS Wrapper."
      fi
      echo "source $REALSENSE_WS/devel/setup.bash" >> ~/.bashrc
    else
      echo "RealSense ROS Wrapper already installed!"
    fi
    source $REALSENSE_WS/devel/setup.bash

    # Step 3: Install apriltag ROS Wrapper
    APRILTAG_WS=~/apriltag_ws
    if [ ! -d "$APRILTAG_WS/src" ]; then
      echo -e "${GRN}Installing Apriltag ROS Wrapper...${OFF}"
      mkdir -p $APRILTAG_WS/src
      cd $APRILTAG_WS/src
      git clone https://github.com/AprilRobotics/apriltag.git
      git clone https://github.com/AprilRobotics/apriltag_ros.git
      cd $APRILTAG_WS
      rosdep install --from-paths src --ignore-src -r -y
      catkin_make_isolated
      if [ $? -eq 0 ]; then
        echo -e "${GRN}${BOLD}Apriltag ROS Wrapper built successfully!${NORM}${OFF}"
      else
        failed "Failed to build Apriltag ROS Wrapper."
      fi
      echo "source $APRILTAG_WS/devel_isolated/setup.bash" >> ~/.bashrc
    else
      echo "Apriltag ROS Wrapper already installed!"
    fi
    source $APRILTAG_WS/devel_isolated/setup.bash
  fi

  # Step 4: Install Arm packages
  if [ ! -d "$INSTALL_PATH/src" ]; then
    echo -e "${GRN}Installing ROS packages for the Interbotix Arm...${OFF}"
    mkdir -p $INSTALL_PATH/src
    cd $INSTALL_PATH/src
    git clone https://github.com/Interbotix/interbotix_ros_core.git -b $ROS_DISTRO_TO_INSTALL
    git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b $ROS_DISTRO_TO_INSTALL
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b $ROS_DISTRO_TO_INSTALL
    rm interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
    rm interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE
    if [ "$INSTALL_PERCEPTION" = true ]; then
      rm interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/CATKIN_IGNORE
      rm interbotix_ros_toolboxes/interbotix_perception_toolbox/CATKIN_IGNORE
    fi
    rm interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
    rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE
    if [ "$INSTALL_MATLAB" = true ]; then
      cd interbotix_ros_toolboxes
      git submodule update --init third_party_libraries/ModernRobotics
      cd ..
    fi
    cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd $INSTALL_PATH
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    if [ $? -eq 0 ]; then
      echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
    else
      failed "Failed to build Interbotix Arm ROS Packages."
    fi
    echo "source $INSTALL_PATH/devel/setup.bash" >> ~/.bashrc
  else
    echo "Interbotix Arm ROS packages already installed!"
  fi
  source $INSTALL_PATH/devel/setup.bash
}

function install_ros2() {
  # Step 1: Install ROS2
  if [ $(dpkg-query -W -f='${Status}' ros-$ROS_DISTRO_TO_INSTALL-desktop 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo -e "${GRN}Installing ROS2 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO_TO_INSTALL-desktop
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    fi
    echo "source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash" >> ~/.bashrc
    sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-colcon-common-extensions
    sudo rosdep init
    rosdep update
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop-full is already installed!"
  fi
  source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash

  if [ "$INSTALL_PERCEPTION" = true ]; then
    # Step 2: Install Realsense packages

    # Step 2A: Install librealsense2
    if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
      echo -e "${GRN}Installing librealsense2...${OFF}"
      sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
      sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main" -u
      version="2.50.0-0~realsense0.6128"

      sudo apt -y install                   \
        librealsense2-udev-rules=${version} \
        librealsense2-dkms                  \
        librealsense2=${version}            \
        librealsense2-gl=${version}         \
        librealsense2-gl-dev=${version}     \
        librealsense2-gl-dbg=${version}     \
        librealsense2-net=${version}        \
        librealsense2-net-dev=${version}    \
        librealsense2-net-dbg=${version}    \
        librealsense2-utils=${version}      \
        librealsense2-dev=${version}        \
        librealsense2-dbg=${version}
      sudo apt-mark hold librealsense2*
    else
      echo "librealsense2 already installed!"
    fi

    # Step 2B: Install realsense2 ROS Wrapper
    REALSENSE_WS=~/realsense_ws
    if [ ! -d "$REALSENSE_WS/src" ]; then
      echo "Installing RealSense ROS Wrapper..."
      mkdir -p $REALSENSE_WS/src
      cd $REALSENSE_WS/src
      git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.0.4
      cd $REALSENSE_WS
      rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
      colcon build
      if [ $? -eq 0 ]; then
        echo -e "${GRN}${BOLD}Realsense ROS Wrapper built successfully!${NORM}${OFF}"
      else
        failed "Failed to build Realsense ROS Wrapper."
      fi
      echo "source $REALSENSE_WS/install/setup.bash" >> ~/.bashrc
    else
      echo "RealSense ROS Wrapper already installed!"
    fi
    source $REALSENSE_WS/install/setup.bash

    # Step 3: Install apriltag ROS Wrapper
    APRILTAG_WS=~/apriltag_ws
    if [ ! -d "$APRILTAG_WS/src" ]; then
      echo "Installing Apriltag ROS Wrapper..."
      mkdir -p $APRILTAG_WS/src
      cd $APRILTAG_WS/src
      git clone https://github.com/AprilRobotics/apriltag.git
      git clone https://github.com/Interbotix/apriltag_ros.git -b ros2-port
      cd $APRILTAG_WS
      rosdep install --from-paths src --ignore-src -r -y
      colcon build
      if [ $? -eq 0 ]; then
        echo -e "${GRN}${BOLD}Apriltag ROS Wrapper built successfully!${NORM}${OFF}"
      else
        failed "Failed to build Apriltag ROS Wrapper."
      fi
      echo "source $APRILTAG_WS/install/setup.bash" >> ~/.bashrc
    else
      echo "Apriltag ROS Wrapper already installed!"
    fi
    source $APRILTAG_WS/install/setup.bash
  fi

  # Step 4: Install Arm packages
  if [ ! -d "$INSTALL_PATH/src" ]; then
    echo -e "${GRN}Installing ROS packages for the Interbotix Arm...${OFF}"
    mkdir -p $INSTALL_PATH/src
    cd $INSTALL_PATH/src
    git clone https://github.com/Interbotix/interbotix_ros_core.git -b $ROS_DISTRO_TO_INSTALL
    git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b $ROS_DISTRO_TO_INSTALL
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b $ROS_DISTRO_TO_INSTALL
    # TODO(lsinterbotix) remove below when moveit_visual_tools is available in apt repo
    git clone https://github.com/ros-planning/moveit_visual_tools.git -b ros2
    if [ "$INSTALL_PERCEPTION" = true ]; then
      rm interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/COLCON_IGNORE
      rm interbotix_ros_toolboxes/interbotix_perception_toolbox/COLCON_IGNORE
    fi
    rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE
    rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE
    cd interbotix_ros_core
    git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox
    git submodule update --init interbotix_ros_xseries/interbotix_xs_driver
    cd ..
    if [ "$INSTALL_MATLAB" = true ]; then
      cd interbotix_ros_toolboxes
      git submodule update --init third_party_libraries/ModernRobotics
      cd ..
    fi
    cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd $INSTALL_PATH
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    if [ $? -eq 0 ]; then
      echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
    else
      failed "Failed to build Interbotix Arm ROS Packages."
    fi
    echo "source $INSTALL_PATH/install/setup.bash" >> ~/.bashrc
  else
    echo "Interbotix Arm ROS packages already installed!"
  fi
  source $INSTALL_PATH/install/setup.bash
}

function setup_env_vars() {
  # Step 5: Setup Environment Variables
  if [ -z "$ROS_IP" ]; then
    echo -e "${GRN}Setting up Environment Variables...${OFF}"
    echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
  else
    echo "Environment variables already set!"
  fi
}

# parse command line arguments
while getopts 'hnd:p:' OPTION;
do
  case "$OPTION" in
    h) help && exit 0;;
    n) NONINTERACTIVE=true;;
    d) ROS_DISTRO_TO_INSTALL="$OPTARG" && DISTRO_SET_FROM_CL=true;;
    p) INSTALL_PATH="$OPTARG";;
    *) echo "Unknown argument $OPTION" && help && exit 0;;
  esac
done
shift "$(($OPTIND -1))"

if ! command -v lsb_release &> /dev/null; then
  sudo apt update
  sudo apt-get install -y lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

# set default ROS distro before reading clargs
if [ "$DISTRO_SET_FROM_CL" = false ]; then
  if [ $UBUNTU_VERSION == "18.04" ]; then
    ROS_DISTRO_TO_INSTALL="melodic"
  elif [ $UBUNTU_VERSION == "20.04" ]; then
    ROS_DISTRO_TO_INSTALL="noetic"
  else
    echo -e "${BOLD}${RED}Unsupported Ubuntu verison: $UBUNTU_VERSION.${NORM}${OFF}"
    failed "Interbotix Arm only works with Ubuntu 18.04 bionic or 20.04 focal on your hardware."
  fi
fi

check_ubuntu_version
validate_distro

if [ "$NONINTERACTIVE" = false ]; then
  # prompt for perecption packages
  echo -e "${BLU}${BOLD}Install the Interbotix Perception packages? This will include the RealSense and AprilTag packages as dependencies.\n$PROMPT${NORM}${OFF}\c"
  read -r resp
  if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
    INSTALL_PERCEPTION=true
  else
    INSTALL_PERCEPTION=false
  fi

  echo -e "${BLU}${BOLD}Install the MATLAB-ROS API?\n$PROMPT${NORM}${OFF}\c"
  read -r resp
  if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
    INSTALL_MATLAB=true
  else
    INSTALL_MATLAB=false
  fi

  echo -e "${BLU}${BOLD}INSTALLATION SUMMARY:"
  echo -e "\tROS Distribution:           ROS${ROS_VERSION_TO_INSTALL} ${ROS_DISTRO_TO_INSTALL}"
  echo -e "\tInstall Perception Modules: ${INSTALL_PERCEPTION}"
  echo -e "\tInstall MATLAB Modules:     ${INSTALL_MATLAB}"
  echo -e "\tInstallation path:          ${INSTALL_PATH}"
  echo -e "\nIs this correct?\n${PROMPT}${NORM}${OFF}\c"
  read -r resp

  if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
    :
  else
    help && exit 0
  fi
else
  INSTALL_PERCEPTION=true
  INSTALL_MATLAB=true
fi

echo -e "\n\n"
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}            Starting installation!            ${NORM}${OFF}"
echo -e "${GRN}${BOLD}   This process may take around 15 Minutes!   ${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo -e "\n\n"

sleep 4
start_time="$(date -u +%s)"

# Update the system
sudo apt update && sudo apt -y upgrade
sudo apt -y autoremove

install_essential_packages

if [[ $ROS_VERSION_TO_INSTALL == 1 ]]; then
  install_ros1
elif [[ $ROS_VERSION_TO_INSTALL == 2 ]]; then
  install_ros2
else
  failed "Something went wrong."
fi

setup_env_vars

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo -e "${GRN}Installation complete, took $elapsed seconds in total.${OFF}"
echo -e "${GRN}NOTE: Remember to reboot the computer before using the robot!${OFF}"
