cmake_minimum_required(VERSION 3.8)
project(dis_homework2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Find all the packages that we are using
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# This is needed if we generate custom interfaces (messages, services or actions)
find_package(rosidl_default_generators REQUIRED)

# A macro, not to repeat the dependencies
set(dependencies rclcpp std_msgs std_srvs rosidl_default_generators)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/MoveService.srv
  DEPENDENCIES std_msgs # Add packages that the listed interfaces depend on
)


#ament_target_dependencies(custom_publisher ${dependencies})

#install(TARGETS
#  custom_publisher
#  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  dis_homework2/move_service.py
  dis_homework2/move_service_old.py
  dis_homework2/go_to_position_simple_node.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(${dependencies} rosidl_default_runtime)

ament_package()
