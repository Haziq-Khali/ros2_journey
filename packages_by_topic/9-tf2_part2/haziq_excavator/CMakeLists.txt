cmake_minimum_required(VERSION 3.8)
project(haziq_excavator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(frame_excavator_tf2_broadcaster src/frame_excavator_tf2_broadcaster.cpp)
ament_target_dependencies(frame_excavator_tf2_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

add_executable(frame_robot_cleaner_broadcaster src/frame_robot_cleaner_broadcaster.cpp)
ament_target_dependencies(frame_robot_cleaner_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

add_executable(ext_frame_robot_cleaner_broadcaster src/ext_frame_robot_cleaner_broadcaster.cpp)
ament_target_dependencies(ext_frame_robot_cleaner_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

add_executable(excavator_listener src/excavator_listener.cpp)
ament_target_dependencies(excavator_listener
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS
  frame_excavator_tf2_broadcaster frame_robot_cleaner_broadcaster excavator_listener
  ext_frame_robot_cleaner_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
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
