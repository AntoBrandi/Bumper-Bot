cmake_minimum_required(VERSION 3.8)
project(bumperbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(dijkstra_planner_costmap src/dijkstra_planner_costmap.cpp)
ament_target_dependencies(dijkstra_planner_costmap rclcpp nav_msgs geometry_msgs tf2_ros)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  dijkstra_planner
  dijkstra_planner_costmap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
