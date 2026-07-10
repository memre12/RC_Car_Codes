cmake_minimum_required(VERSION 3.5)
project(v_ref_gen)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# Create executable
add_executable(v_ref_gen src/path_publisher.cpp)
ament_target_dependencies(v_ref_gen rclcpp geometry_msgs nav_msgs)

add_executable(desired_speed_pub src/desired_speed_calc.cpp)
ament_target_dependencies(desired_speed_pub   
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

# Install target
install(TARGETS
  v_ref_gen
  desired_speed_pub
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
  )
ament_package()
