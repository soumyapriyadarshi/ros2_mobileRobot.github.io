
cmake_minimum_required(VERSION 3.10)

project(potential_field_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Executable
add_executable(potential_field_node
  src/potential_field_node.cpp
)

# Modern ROS 2 Lyrical dependencies
target_link_libraries(potential_field_node PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

# Install
install(
  TARGETS potential_field_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

