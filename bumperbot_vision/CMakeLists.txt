cmake_minimum_required(VERSION 3.8)
project(bumperbot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  image_transport
  point_cloud_transport
  message_filters
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(stereo_vision
  src/stereo_vision.cpp
)
ament_target_dependencies(stereo_vision ${dependencies})

add_executable(visual_odometry
  src/visual_odometry.cpp
)
ament_target_dependencies(visual_odometry ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  stereo_vision
  visual_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/stereo_vision.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
