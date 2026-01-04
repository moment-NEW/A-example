cmake_minimum_required(VERSION 3.8)
project(a_star_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(a_star_node src/a_star_node.cpp)
ament_target_dependencies(a_star_node rclcpp nav_msgs geometry_msgs)

target_compile_options(a_star_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS a_star_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
