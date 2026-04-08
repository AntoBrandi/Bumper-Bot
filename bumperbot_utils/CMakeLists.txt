cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(trajectory_drawer src/trajectory_drawer.cpp)
ament_target_dependencies(trajectory_drawer rclcpp geometry_msgs nav_msgs)

add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp std_msgs sensor_msgs geometry_msgs twist_mux_msgs rclcpp_action visualization_msgs)

add_executable(webcam_publisher src/webcam_publisher.cpp)
target_include_directories(webcam_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(webcam_publisher rclcpp sensor_msgs cv_bridge OpenCV)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  trajectory_drawer
  safety_stop
  webcam_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/trajectory_drawer.py
  ${PROJECT_NAME}/safety_stop.py
  ${PROJECT_NAME}/webcam_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
