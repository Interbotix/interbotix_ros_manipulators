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

ALL_VALID_DISTROS=('melodic' 'noetic' 'galactic' 'humble' 'rolling')
ROS1_VALID_DISTROS=('melodic' 'noetic')
ROS2_VALID_DISTROS=('galactic' 'humble' 'rolling')

BIONIC_VALID_DISTROS=('melodic')
FOCAL_VALID_DISTROS=('noetic' 'galactic')
JAMMY_VALID_DISTROS=('humble' 'rolling')

NONINTERACTIVE=false
DISTRO_SET_FROM_CL=false
INSTALL_PATH=~/interbotix_ws
APRILTAG_WS=~/apriltag_ws

_usage="${BOLD}USAGE: ./xsarm_amd64_install.sh [-h][-d DISTRO][-p PATH][-n]${NORM}

Install the Interbotix X-Series Arms packages and their dependencies.

Options:

  -h              Display this help message and quit

  -d DISTRO       Install the DISTRO ROS distro compatible with your Ubuntu version. See
                  'https://github.com/Interbotix/.github/blob/main/SECURITY.md' for the list of
                  supported distributions. If not given, installs the ROS 1 distro compatible with
                  your Ubuntu version, or the stable ROS 2 distro if using Ubuntu 22.04 or later.

  -p PATH         Sets the absolute install location for the Interbotix workspace. If not specified,
                  the Interbotix workspace directory will default to '~/interbotix_ws'.

  -n              Install all packages and dependencies without prompting. This is useful if
                  you're running this script in a non-interactive terminal like when building a
                  Docker image.

Examples:

  ./xsarm_amd64_install.sh ${BOLD}-h${NORM}
    This will display this help message and quit.

  ./xsarm_amd64_install.sh
    This will install just the ROS 1 distro compatible with your Ubuntu version, or the stable ROS 2
    distro if using Ubuntu 22.04 or later. It will prompt you to ask if you want to install certain
    packages and dependencies.

  ./xsarm_amd64_install.sh ${BOLD}-d noetic${NORM}
    This will install ROS 1 Noetic assuming that your Ubuntu version is compatible.

  ./xsarm_amd64_install.sh ${BOLD}-n${NORM}
    Skip prompts and install all packages and dependencies.

  ./xsarm_amd64_install.sh ${BOLD}-d galactic${NORM}
    Install ROS 2 Galactic assuming that your Ubuntu version is compatible.

  ./xsarm_amd64_install.sh ${BOLD}-d galactic -n${NORM}
    Install ROS 2 Galactic and all packages and dependencies.

  ./xsarm_amd64_install.sh ${BOLD}-p ~/custom_ws${NORM}
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
  if contains_element "$ROS_DISTRO_TO_INSTALL" "${ALL_VALID_DISTROS[@]}"; then
    if contains_element "$ROS_DISTRO_TO_INSTALL" "${ROS1_VALID_DISTROS[@]}"; then
      # Supported ROS 1 distros
      ROS_VERSION_TO_INSTALL=1
    elif contains_element "$ROS_DISTRO_TO_INSTALL" "${ROS2_VALID_DISTROS[@]}"; then
      # Supported ROS 2 distros
      ROS_VERSION_TO_INSTALL=2
    else
      # For cases where it passes the first check but somehow fails the second check
      failed "Something went wrong. ROS_DISTRO_TO_INSTALL=$ROS_DISTRO_TO_INSTALL."
    fi
    echo -e "${GRN}${BOLD}Chosen Version: ROS ${ROS_VERSION_TO_INSTALL} $ROS_DISTRO_TO_INSTALL${NORM}${OFF}"
    return 0
  else
    failed "'$ROS_DISTRO_TO_INSTALL' is not a valid ROS Distribution. Choose one of: '${ALL_VALID_DISTROS[*]}'"
  fi
}

function check_ubuntu_version() {
 # check if the chosen distribution is compatible with the Ubuntu version
  case $UBUNTU_VERSION in

    18.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${BIONIC_VALID_DISTROS[@]}"; then
        PY_VERSION=2
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    20.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${FOCAL_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    22.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${JAMMY_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    *)
      failed "Something went wrong. UBUNTU_VERSION='$UBUNTU_VERSION', should be 18.04, 20.04, or 22.04."
      ;;

  esac
}

function install_essential_packages() {
  # Install necessary core packages
  sudo apt-get install -yq curl git
  if [ "$ROS_VERSION_TO_INSTALL" == 2 ]; then
    sudo pip3 install transforms3d
  fi
  if [ $PY_VERSION == 2 ]; then
    sudo apt-get install -yq python-pip
    python -m pip install modern_robotics
  elif [ $PY_VERSION == 3 ]; then
    sudo apt-get install -yq python3-pip
    python3 -m pip install modern_robotics
  else
    failed "Something went wrong. PY_VERSION='$PY_VERSION', should be 2 or 3."
  fi
}

function install_ros1() {
  # Install ROS 1
  if [ "$(dpkg-query -W -f='${Status}' ros-"$ROS_DISTRO_TO_INSTALL"-desktop-full 2>/dev/null | grep -c "ok installed")" -eq 0 ]; then
    echo -e "${GRN}Installing ROS 1 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install -yq ros-"$ROS_DISTRO_TO_INSTALL"-desktop-full
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    fi
    echo "source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash" >> ~/.bashrc
    if [ $PY_VERSION == 2 ]; then
      sudo apt-get install -yq          \
        python-rosdep                   \
        python-rosinstall               \
        python-rosinstall-generator     \
        python-wstool                   \
        build-essential
    elif [ $PY_VERSION == 3 ]; then
      sudo apt-get install -yq          \
        python3-rosdep                  \
        python3-rosinstall              \
        python3-rosinstall-generator    \
        python3-wstool                  \
        build-essential
    fi
    sudo rosdep init
    rosdep update --include-eol-distros
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop-full is already installed!"
  fi
  source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash

  # Install Arm packages
  if source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash 2>/dev/null && \
     source "$INSTALL_PATH"/devel/setup.bash 2>/dev/null && \
     rospack list | grep -q interbotix_;
  then
    echo "Interbotix Arm ROS packages already installed!"
  else
    cd "$INSTALL_PATH"/src
    git clone -b "$ROS_DISTRO_TO_INSTALL" https://github.com/Interbotix/interbotix_ros_core.git
    git clone -b "$ROS_DISTRO_TO_INSTALL" https://github.com/Interbotix/interbotix_ros_manipulators.git
    git clone -b "$ROS_DISTRO_TO_INSTALL" https://github.com/Interbotix/interbotix_ros_toolboxes.git
    rm                                                                                              \
      interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE                                      \
      interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE                               \
      interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE                                  \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE
    if [ "$INSTALL_PERCEPTION" = true ]; then
      rm                                                                                            \
        interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/CATKIN_IGNORE \
        interbotix_ros_toolboxes/interbotix_perception_toolbox/CATKIN_IGNORE
    fi
    if [ "$INSTALL_MATLAB" = true ]; then
      cd interbotix_ros_toolboxes
      git submodule update --init third_party_libraries/ModernRobotics
      cd "$INSTALL_PATH/src"
    fi
    cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd "$INSTALL_PATH"
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    if catkin_make; then
      echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
      echo "source $INSTALL_PATH/devel/setup.bash" >> ~/.bashrc
      source "$INSTALL_PATH"/devel/setup.bash
    else
      failed "Failed to build Interbotix Arm ROS Packages."
    fi
  fi
}

function install_ros2() {
  # Install ROS 2
  if [ "$(dpkg-query -W -f='${Status}' ros-"$ROS_DISTRO_TO_INSTALL"-desktop 2>/dev/null | grep -c "ok installed")" -eq 0 ]; then
    echo -e "${GRN}Installing ROS 2 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    sudo apt-get install -yq      \
      software-properties-common  \
      gnupg
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -yq ros-"$ROS_DISTRO_TO_INSTALL"-desktop
    echo "source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash" >> ~/.bashrc
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop-full is already installed!"
  fi
  source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash

  # Install rosdep and other necessary tools
  sudo apt-get install -yq            \
    python3-rosdep                    \
    python3-rosinstall                \
    python3-rosinstall-generator      \
    python3-wstool                    \
    build-essential                   \
    python3-colcon-common-extensions

  # Remove sources if they exist
  if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
  fi

  # Initialize rosdep sources
  sudo rosdep init

  # Update local rosdep database, including EoL distros
  rosdep update --include-eol-distros

  if [ "$INSTALL_PERCEPTION" = true ]; then
    # Install apriltag ROS Wrapper, no official Apriltag ROS 2 package yet
    if source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash 2>/dev/null && \
      source "$APRILTAG_WS"/install/setup.bash 2>/dev/null && \
      ros2 pkg list | grep -q apriltag_ros;
    then
      echo "Apriltag ROS Wrapper already installed!"
    else
      echo -e "${GRN}Installing Apriltag ROS Wrapper...${OFF}"
      mkdir -p "$APRILTAG_WS"/src
      cd "$APRILTAG_WS"/src
      git clone -b ros2-port https://github.com/Interbotix/apriltag_ros.git
      cd "$APRILTAG_WS"
      rosdep install --from-paths src --ignore-src -r -y
      # cmake-args flags disables warnings as errors unrelated to ROS
      if colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"; then
        echo -e "${GRN}${BOLD}Apriltag ROS Wrapper built successfully!${NORM}${OFF}"
        echo "source $APRILTAG_WS/install/setup.bash" >> ~/.bashrc
        source $APRILTAG_WS/install/setup.bash
      else
        failed "Failed to build Apriltag ROS Wrapper."
      fi
    fi
  fi

  # Install Arm packages
  if source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash 2>/dev/null && \
     source "$INSTALL_PATH"/install/setup.bash 2>/dev/null && \
     ros2 pkg list | grep -q interbotix_;
  then
    echo "Interbotix Arm ROS 2 packages already installed!"
  else
    echo -e "${GRN}Installing ROS 2 packages for the Interbotix Arm...${OFF}"
    cd "$INSTALL_PATH"/src
    git clone -b "$ROS_DISTRO_TO_INSTALL" https://github.com/Interbotix/interbotix_ros_core.git
    git clone -b "$ROS_DISTRO_TO_INSTALL" https://github.com/Interbotix/interbotix_ros_manipulators.git
    git clone -b "$ROS_DISTRO_TO_INSTALL" https://github.com/Interbotix/interbotix_ros_toolboxes.git
    # TODO(lsinterbotix) remove below when moveit_visual_tools is available in apt repo
    git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools.git
    if [ "$INSTALL_PERCEPTION" = true ]; then
      rm                                                                                                \
        interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/COLCON_IGNORE     \
        interbotix_ros_toolboxes/interbotix_perception_toolbox/COLCON_IGNORE
    fi
    rm                                                                                                  \
      interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE                                          \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE      \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE
    cd interbotix_ros_core
    git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox
    git submodule update --init interbotix_ros_xseries/interbotix_xs_driver
    cd "$INSTALL_PATH"/src
    if [ "$INSTALL_MATLAB" = true ]; then
      cd interbotix_ros_toolboxes
      git submodule update --init third_party_libraries/ModernRobotics
      cd "$INSTALL_PATH"/src
    fi
    cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd "$INSTALL_PATH"
    rosdep install --from-paths src --ignore-src -r -y
    if colcon build; then
      echo -e "${GRN}${BOLD}Interbotix Arm ROS 2 Packages built successfully!${NORM}${OFF}"
      echo "source $INSTALL_PATH/install/setup.bash" >> ~/.bashrc
      source "$INSTALL_PATH"/install/setup.bash
    else
      failed "Failed to build Interbotix Arm ROS 2 Packages."
    fi
  fi
}

function setup_env_vars() {
  # Set up Environment Variables
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

shift "$((OPTIND -1))"

if ! command -v lsb_release &> /dev/null; then
  sudo apt-get update
  sudo apt-get install -yq lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

# set default ROS distro before reading clargs
if [ "$DISTRO_SET_FROM_CL" = false ]; then
  if [ "$UBUNTU_VERSION" == "18.04" ]; then
    ROS_DISTRO_TO_INSTALL="melodic"
  elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    ROS_DISTRO_TO_INSTALL="noetic"
  elif [ "$UBUNTU_VERSION" == "22.04" ]; then
    ROS_DISTRO_TO_INSTALL="humble"
  else
    echo -e "${BOLD}${RED}Unsupported Ubuntu version: $UBUNTU_VERSION.${NORM}${OFF}"
    failed "Interbotix Arm only works with Ubuntu 18.04 Bionic, 20.04 Focal, or 22.04 Jammy on your hardware."
  fi
fi

validate_distro
check_ubuntu_version

if [ "$NONINTERACTIVE" = false ]; then
  # prompt for perception packages
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

  if [[ "$ROS_DISTRO_TO_INSTALL" == 'rolling' ]]; then
    echo -e "${BLU}${BOLD}ROS 2 Rolling is not officially supported and full functionality is not guaranteed. Do you wish to continue anyways?\n$PROMPT${NORM}${OFF}\c"
    read -r resp
    if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
      :
    else
      help && exit 0
    fi
  fi

  echo -e "${BLU}${BOLD}INSTALLATION SUMMARY:"
  echo -e "\tROS Distribution:           ROS ${ROS_VERSION_TO_INSTALL} ${ROS_DISTRO_TO_INSTALL}"
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

echo -e "\n# Interbotix Configurations" >> ~/.bashrc

# Update the system
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get -y autoremove

install_essential_packages

mkdir -p "$INSTALL_PATH"/src

shopt -s extglob

if [[ $ROS_VERSION_TO_INSTALL == 1 ]]; then
  install_ros1
elif [[ $ROS_VERSION_TO_INSTALL == 2 ]]; then
  install_ros2
else
  failed "Something went wrong."
fi

setup_env_vars

shopt -u extglob

end_time="$(date -u +%s)"
elapsed="$((end_time-start_time))"

echo -e "${GRN}Installation complete, took $elapsed seconds in total.${OFF}"
echo -e "${GRN}NOTE: Remember to reboot the computer before using the robot!${OFF}"
