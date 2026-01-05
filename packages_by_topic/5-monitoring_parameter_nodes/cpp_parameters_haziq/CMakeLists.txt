cmake_minimum_required(VERSION 3.8)
project(cpp_parameters_haziq)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_param_haziq src/cpp_param_haziq.cpp)
ament_target_dependencies(cpp_param_haziq rclcpp)
install(TARGETS 
cpp_param_haziq
DESTINATION lib/${PROJECT_NAME}
)

add_executable(cpp_param_haziq_limit src/cpp_param_haziq_limit.cpp)
ament_target_dependencies(cpp_param_haziq_limit rclcpp)
install(TARGETS
cpp_param_haziq_limit
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
