cmake_minimum_required(VERSION 3.5)
project(interbotix_ros_xsarms)
find_package(ament_cmake REQUIRED)
unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
