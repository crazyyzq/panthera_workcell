cmake_minimum_required(VERSION 3.8)
project(panthera_rs485)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(panthera_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(panthera_rs485_core
  src/serial_port.cpp
  src/modbus_rtu.cpp
  src/laser_displacement_sensor.cpp
)

target_include_directories(panthera_rs485_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(panthera_rs485_core
  rclcpp
)

add_executable(laser_distance_node
  src/laser_distance_node.cpp
)
target_link_libraries(laser_distance_node panthera_rs485_core)
ament_target_dependencies(laser_distance_node
  rclcpp
  geometry_msgs
  panthera_interfaces
)

add_executable(rs485_state_signal_node
  src/rs485_state_signal_node.cpp
)
target_link_libraries(rs485_state_signal_node panthera_rs485_core)
ament_target_dependencies(rs485_state_signal_node
  rclcpp
  panthera_interfaces
)

install(TARGETS
  panthera_rs485_core
  laser_distance_node
  rs485_state_signal_node
  EXPORT panthera_rs485_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(panthera_rs485_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
