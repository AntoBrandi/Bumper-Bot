cmake_minimum_required(VERSION 3.8)
project(bumperbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(dependencies
  rclcpp
  image_transport
  message_filters
  sensor_msgs
  geometry_msgs
  cv_bridge
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ball_follower
  src/ball_follower.cpp
)
ament_target_dependencies(ball_follower ${dependencies})
target_link_libraries(ball_follower
  ${OpenCV_LIBS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ball_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/ball_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_tree config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
