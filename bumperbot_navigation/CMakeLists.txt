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
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(include)

add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(dijkstra_planner_costmap src/dijkstra_planner_costmap.cpp)
ament_target_dependencies(dijkstra_planner_costmap rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(a_star_planner_costmap src/a_star_planner_costmap.cpp)
ament_target_dependencies(a_star_planner_costmap rclcpp nav_msgs geometry_msgs tf2_ros)

add_library(dijkstra_planner_plugin SHARED src/nav2_dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner_plugin
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
)

add_library(a_star_planner_plugin SHARED src/nav2_a_star_planner.cpp)
ament_target_dependencies(a_star_planner_plugin
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS dijkstra_planner_plugin a_star_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  dijkstra_planner
  dijkstra_planner_costmap
  a_star_planner
  a_star_planner_costmap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(dijkstra_planner_plugin a_star_planner_plugin)
ament_package()
