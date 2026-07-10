cmake_minimum_required(VERSION 3.8)
project(race_car_controller)

# Default to an optimized build: without this, colcon builds with no
# optimization at all, which is far too slow on the Jetson Xavier.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Find dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# === Include header files ===
include_directories(include)

# === Add executable ===
add_executable(race_car_controller_node src/race_car_controller.cpp)

# === Link dependencies ===
ament_target_dependencies(race_car_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  vesc_msgs
  visualization_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

# === Header path for installed targets ===
target_include_directories(race_car_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# === Install executable ===
install(TARGETS
  race_car_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Install header files ===
install(DIRECTORY include/
  DESTINATION include/
)

# === Install launch/config folders (optional but recommended) ===
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# === Linting and tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
