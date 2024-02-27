cmake_minimum_required(VERSION 3.5)
project(interbotix_xsarm_puppet)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ROS_DEPENDENCIES
  interbotix_xs_msgs
  rclcpp
  sensor_msgs
)

include_directories(
  include
)

add_executable(xsarm_puppet src/xsarm_puppet.cpp)
ament_target_dependencies(xsarm_puppet ${ROS_DEPENDENCIES})

add_executable(xsarm_puppet_single src/xsarm_puppet_single.cpp)
ament_target_dependencies(xsarm_puppet_single ${ROS_DEPENDENCIES})

install(
  TARGETS
    xsarm_puppet
    xsarm_puppet_single
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    bag
    config
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
