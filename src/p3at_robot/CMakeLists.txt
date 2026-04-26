cmake_minimum_required(VERSION 3.8)
project(p3at_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  src/gamepad_controller.py
  src/gps_waypoint_navigator.py
  src/mag_calibrate.py
  src/nav2_waypoints.py
  src/phidget_imu.py
  src/oak_camera.py
  src/imu_calibrate.py
  src/cone_detection.py
  src/object_detector.py
  src/waypoint_scanner.py
  src/test_oak_depth.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(ariaNode src/ariaNode.cpp)
ament_target_dependencies(ariaNode
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2
)
target_link_libraries(ariaNode libAria.so)

install(TARGETS ariaNode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
