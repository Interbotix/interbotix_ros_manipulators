cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsarm_moveit_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interbotix_xsarm_moveit
  interbotix_moveit_interface
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 CATKIN_DEPENDS interbotix_xsarm_moveit interbotix_moveit_interface
)
