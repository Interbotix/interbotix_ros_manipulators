#!/usr/bin/env bash

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'
ORG='\033[;33m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

PROMPT="> "

INSTALL_PATH=~/interbotix_ws

_usage="${BOLD}USAGE: ./xsarm_remote_install.sh [-h][-p PATH]${NORM}

Install the Interbotix X-Series Arms remote packages and their dependencies.

Note that the remote install option has been removed for ROS 2 distributions for now. Please use
the full install script corresponding to your hardware's architecture.

Options:

  -h              Display this help message and quit

  -p PATH         Sets the absolute install location for the Interbotix workspace. If not specified,
                  the Interbotix workspace directory will default to '~/interbotix_ws'.

Examples:

  ./xsarm_remote_install.sh ${BOLD}-h${NORM}
    This will display this help message and quit.

  ./xsarm_remote_install.sh
    This will install just the ROS 1 distro compatible with your Ubuntu version. It will prompt you
    to ask if you want to install certain packages and dependencies.

  ./xsarm_remote_install.sh ${BOLD}-p ~/custom_ws${NORM}
    Installs the Interbotix packages under the '~/custom_ws' path."

function help() {
  # print usage
  cat << EOF
$_usage
EOF
}

function failed() {
  # Log error and quit with a failed exit code
  echo -e "${ERR}[ERROR] $1${RRE}"
  echo -e "${ERR}[ERROR] Interbotix Remote Installation Failed!${RRE}"
  exit 1
}

# parse command line arguments
while getopts 'hp:' OPTION;
do
  case "$OPTION" in
    h) help && exit 0;;
    p) INSTALL_PATH="$OPTARG";;
    *) echo "Unknown argument $OPTION" help && exit 0;;
  esac
done

shift "$((OPTIND -1))"

if ! command -v lsb_release &> /dev/null; then
  sudo apt-get update
  sudo apt-get install -yq lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

if [ "$UBUNTU_VERSION" == "18.04" ]; then
  ROS_DISTRO_TO_INSTALL="melodic"
elif [ "$UBUNTU_VERSION" == "20.04" ]; then
  ROS_DISTRO_TO_INSTALL="noetic"
else
  echo -e "${BOLD}${RED}Unsupported Ubuntu version: $UBUNTU_VERSION.${NORM}${OFF}"
  failed "Interbotix Remote Arm only works with Ubuntu 18.04 Bionic or 20.04 Focal on your hardware."
fi

echo -e "${BLU}${BOLD}What is the hostname of the robot computer (type 'hostname' in a terminal to find it)?\n$PROMPT${NORM}${OFF}\c"
read -r HOSTNAME

echo -e "${BLU}${BOLD}REMOTE INSTALLATION SUMMARY:"
echo -e "\tROS Distribution:           ROS ${ROS_VERSION_TO_INSTALL} ${ROS_DISTRO_TO_INSTALL}"
echo -e "\tRemote Hostname:            ${HOSTNAME}"
echo -e "\tInstallation path:          ${INSTALL_PATH}"
echo -e "\nIs this correct?\n${PROMPT}${NORM}${OFF}\c"
read -r resp

if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
  :
else
  help && exit 0
fi

echo -e "\n\n"
echo -e "${GRN}${BOLD}********************************************${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}          Starting installation!            ${NORM}${OFF}"
echo -e "${GRN}${BOLD}   This process may take a few minutes!     ${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}********************************************${NORM}${OFF}"
echo -e "\n\n"

sleep 4
start_time="$(date -u +%s)"

# Install Arm packages
if [ ! -d "$INSTALL_PATH/src" ]; then
  echo -e "${GRN}Installing Simulation/Visualization ROS packages for the Interbotix Arms...${OFF}"
  source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash
  mkdir -p "$INSTALL_PATH"/src
  cd "$INSTALL_PATH"
  catkin_make
  cd src
  git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b $ROS_DISTRO_TO_INSTALL
  rm interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE
  echo "source $INSTALL_PATH/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix Arm Remote ROS packages already installed!"
fi
source "$INSTALL_PATH"/devel/setup.bash


# Update the system
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get -y autoremove

sudo apt-get install -yq openssh-server

# Setup Environment Variables
if [ -z "$ROS_IP" ]; then
  echo -e "${GRN}Setting up Environment Variables...${OFF}"
  echo -e "\n# Interbotix Configurations" >> ~/.bashrc
  echo "export ROS_MASTER_URI=http://$HOSTNAME.local:11311" >> ~/.bashrc
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi

end_time="$(date -u +%s)"
elapsed="$((end_time-start_time))"

echo -e "${GRN}Installation complete, took $elapsed seconds in total. Close this terminal and open a new one to finish.${OFF}"
echo -e "${GRN}NOTE: Remember to comment out the ${ORG}'source $INSTALL_PATH/devel/setup.bash'${GRN} and ${ORG}'export ROS_MASTER_URI=http://$HOSTNAME.local:11311'${GRN} lines from the ~/.bashrc file when done using the Arm! Then close out of your terminal and open a new one.${OFF}"
