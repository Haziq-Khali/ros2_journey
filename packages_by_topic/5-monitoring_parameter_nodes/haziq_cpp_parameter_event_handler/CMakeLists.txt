cmake_minimum_required(VERSION 3.8)
project(haziq_cpp_parameter_event_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(singleparam_node_event_handler src/singleparam_node_event_handler.cpp)
ament_target_dependencies(singleparam_node_event_handler rclcpp)
install(TARGETS
singleparam_node_event_handler
DESTINATION lib/${PROJECT_NAME}
)

add_executable(manyparam_node_event_handler src/manyparam_node_event_handler.cpp)
ament_target_dependencies(manyparam_node_event_handler rclcpp)
install(TARGETS
manyparam_node_event_handler
DESTINATION lib/${PROJECT_NAME}
)

add_executable(multiple_node_event_handler src/multiple_node_event_handler.cpp)
ament_target_dependencies(multiple_node_event_handler rclcpp)
install(TARGETS
multiple_node_event_handler
DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
