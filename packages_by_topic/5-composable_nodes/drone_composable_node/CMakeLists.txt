cmake_minimum_required(VERSION 3.8)
project(drone_composable_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)



# Build components
## Drone Altitude Component
add_library(drone_altitude_component SHARED
  src/drone_altitude.cpp
)
ament_target_dependencies(drone_altitude_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(drone_altitude_component PLUGIN "drone_altitude::DroneAltitude" EXECUTABLE drone_altittude_comp)

## Drone Sensor Component
add_library(drone_sensor_component SHARED
  src/drone_sensor.cpp
)
ament_target_dependencies(drone_sensor_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(drone_sensor_component PLUGIN "drone_sensor::DroneSensor" EXECUTABLE drone_sensor_comp)

## Drone Rotation Component
add_library(drone_rotation_component SHARED
  src/drone_rotation.cpp
)
ament_target_dependencies(drone_rotation_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(drone_rotation_component PLUGIN "drone_rotation::DroneRotation" EXECUTABLE drone_rotation_comp)

# (Optional) Executable for testing outside composition
#add_executable(drone_rotation_node src/drone_rotation.cpp)
#ament_target_dependencies(drone_rotation_node rclcpp std_msgs)
#target_compile_definitions(drone_rotation_node PRIVATE "COMPOSITION__NODE_BUILDING_EXECUTABLE")

# Install targets
install(TARGETS
  drone_altitude_component
  drone_sensor_component
  drone_rotation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install(TARGETS
#  drone_rotation_node
# DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY include/
  DESTINATION include
)

# Package
ament_package()
