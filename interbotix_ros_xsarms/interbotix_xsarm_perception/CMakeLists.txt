cmake_minimum_required(VERSION 3.5)
project(interbotix_xsarm_perception)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    demos/block_organizer.py
    demos/chessboard_cleanup.py
    demos/color_sorter.py
    demos/pick_place.py
    demos/size_organizer.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
