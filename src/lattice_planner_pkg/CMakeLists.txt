cmake_minimum_required(VERSION 3.8)
project(lattice_planner_pkg)

# Default to an optimized build: without this, colcon builds with no
# optimization at all, which is far too slow on the Jetson Xavier.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17 or newer (allow user to override with a newer standard)
if(NOT DEFINED CMAKE_CXX_STANDARD OR CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Lattice Planner Node
add_executable(lattice_planner_node src/lattice_planner_node.cpp)
ament_target_dependencies(lattice_planner_node
  rclcpp nav_msgs sensor_msgs geometry_msgs visualization_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)

# Mission Planner Node
add_executable(mission_planner src/mission_planner.cpp)
ament_target_dependencies(mission_planner
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs
)

# Scan To Grid Node
add_executable(scan_to_grid src/scan_to_grid.cpp)
ament_target_dependencies(scan_to_grid
  rclcpp nav_msgs sensor_msgs geometry_msgs
)

install(TARGETS
  lattice_planner_node
  mission_planner
  scan_to_grid
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
