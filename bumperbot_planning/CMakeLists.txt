cmake_minimum_required(VERSION 3.8)
project(bumperbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner ${dependencies})

add_executable(dijkstra_costmap_planner src/dijkstra_costmap_planner.cpp)
ament_target_dependencies(dijkstra_costmap_planner ${dependencies})

add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner ${dependencies})

add_executable(a_star_costmap_planner src/a_star_costmap_planner.cpp)
ament_target_dependencies(a_star_costmap_planner ${dependencies})

add_library(dijkstra_planner_plugin SHARED src/nav2_dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner_plugin ${dependencies})

add_library(a_star_planner_plugin SHARED src/nav2_a_star_planner.cpp)
ament_target_dependencies(a_star_planner_plugin ${dependencies})

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

install(TARGETS
  dijkstra_planner_plugin
  a_star_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  dijkstra_planner
  dijkstra_costmap_planner
  a_star_planner
  a_star_costmap_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  dijkstra_planner_plugin
  a_star_planner_plugin
)
ament_export_dependencies(${dependencies})
ament_package()
