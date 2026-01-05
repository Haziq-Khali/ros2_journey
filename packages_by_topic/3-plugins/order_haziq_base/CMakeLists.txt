cmake_minimum_required(VERSION 3.8)
project(order_haziq_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)                                   ##ADDED

add_executable(order_node src/order_node.cpp)
target_include_directories(order_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(order_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  order_node
  "pluginlib"
  "rclcpp"                                                      ##ADDED
)


add_executable(order_node_latch src/order_node_latch.cpp)             # ADDED for order_node_latch
target_include_directories(order_node_latch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(order_node_latch PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  order_node_latch
  "pluginlib"
  rclcpp
)




install(                                                        ##ADDED because we have header interfaces
  DIRECTORY include/                                            ##or else the plugin cannot include the interfaces.
  DESTINATION include
)



install(TARGETS order_node order_node_latch                     ##ADDED here if new nodes
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

ament_export_include_directories(include)                     ##MUST have to export!!!!!

ament_package()
