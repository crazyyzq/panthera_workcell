cmake_minimum_required(VERSION 3.8)
project(panthera_spectrometer_cell)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(panthera_interfaces REQUIRED)
find_package(panthera_rs485 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(spectrometer_cell_node
  src/main.cpp
  src/Config.cpp
  src/LaserProcessing.cpp
  src/RobotActions.cpp
  src/Sensors.cpp
  src/Spectrometer.cpp
  src/StateMachine.cpp
)

add_executable(recover_current_cycle
  src/recover_current_cycle.cpp
  src/Config.cpp
  src/RobotActions.cpp
)

target_include_directories(spectrometer_cell_node
  PRIVATE
    include
)

target_include_directories(recover_current_cycle
  PRIVATE
    include
)

ament_target_dependencies(spectrometer_cell_node
  ament_index_cpp
  control_msgs
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  moveit_ros_planning_interface
  panthera_interfaces
)

ament_target_dependencies(recover_current_cycle
  ament_index_cpp
  control_msgs
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  moveit_ros_planning_interface
  panthera_interfaces
)

target_link_libraries(spectrometer_cell_node panthera_rs485::panthera_rs485_core)
target_link_libraries(recover_current_cycle panthera_rs485::panthera_rs485_core)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(spectrometer_cell_node yaml-cpp::yaml-cpp)
  target_link_libraries(recover_current_cycle yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(spectrometer_cell_node yaml-cpp)
  target_link_libraries(recover_current_cycle yaml-cpp)
else()
  target_link_libraries(spectrometer_cell_node ${YAML_CPP_LIBRARIES})
  target_link_libraries(recover_current_cycle ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS
  spectrometer_cell_node
  recover_current_cycle
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  config/spectrometer_cell.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_sensors
    test/test_sensors.cpp
    src/Sensors.cpp
    src/LaserProcessing.cpp
  )
  target_include_directories(test_sensors PRIVATE include)
  ament_target_dependencies(test_sensors
    rclcpp
    std_srvs
    panthera_interfaces
  )
  set_tests_properties(test_sensors PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=232;ROS_LOCALHOST_ONLY=1"
  )
  ament_add_gtest(test_laser_processing
    test/test_laser_processing.cpp
    src/LaserProcessing.cpp
  )
  target_include_directories(test_laser_processing PRIVATE include)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE C++)
  # ponytail: keep lint enforcing the maintained files without a noisy legacy-only reformat.
  set(ament_cmake_uncrustify_ADDITIONAL_EXCLUDE
    include/panthera_spectrometer_cell/RobotActions.h
    src/Config.cpp
    src/RobotActions.cpp
    src/main.cpp
    src/recover_current_cycle.cpp
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
