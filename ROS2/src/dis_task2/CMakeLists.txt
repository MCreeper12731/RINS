cmake_minimum_required(VERSION 3.8)
project(dis_task2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoverMessage.msg"
  DEPENDENCIES geometry_msgs # Add packages that the listed interfaces depend on
)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/detect_people.py
  scripts/robot_commander.py
  scripts/face_locator.py
  scripts/sweeper.py
  scripts/detect_rings.py
  scripts/ring_locator.py
  scripts/mover.py
  scripts/StereoDepthNode.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies} rosidl_default_runtime)

ament_package()
