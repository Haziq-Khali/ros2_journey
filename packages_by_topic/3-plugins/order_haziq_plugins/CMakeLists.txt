cmake_minimum_required(VERSION 3.8)
project(order_haziq_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(order_haziq_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)                                       ##ADDED


add_library(${PROJECT_NAME}                                         
  src/order_haziq_plugins.cpp                                       ##Use the original plugins file.
  src/calculator_plugins.cpp                                        ##ADDED because created two new plugins file
  src/set_plugins.cpp
)


target_compile_features(order_haziq_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(order_haziq_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  order_haziq_base
  pluginlib
  rclcpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(order_haziq_plugins PRIVATE "ORDER_HAZIQ_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/                    ##NO NEED because we depend on base interfaces, here have no headers
  DESTINATION include
)


install(
  TARGETS order_haziq_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  order_haziq_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# Export plugin description XML                                                   ##ADDED 
pluginlib_export_plugin_description_file(order_haziq_base order_plugins.xml)      ##MUST be the base packages name

# Install the XML file                                                            ##ADDED
install(FILES order_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
