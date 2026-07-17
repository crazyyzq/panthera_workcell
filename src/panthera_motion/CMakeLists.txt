cmake_minimum_required(VERSION 3.8)
project(panthera_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(panthera_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(panthera_motion_core
  src/Catalog.cpp
  src/Compiler.cpp
)
target_include_directories(panthera_motion_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(panthera_motion_core
  moveit_core
  moveit_ros_planning
  rclcpp
  trajectory_msgs
)
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(panthera_motion_core yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(panthera_motion_core yaml-cpp)
else()
  target_link_libraries(panthera_motion_core ${YAML_CPP_LIBRARIES})
endif()

add_executable(motion_server_node src/motion_server_node.cpp)
target_link_libraries(motion_server_node panthera_motion_core)
ament_target_dependencies(motion_server_node
  ament_index_cpp
  control_msgs
  panthera_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
  trajectory_msgs
)

install(
  TARGETS panthera_motion_core motion_server_node
  EXPORT export_panthera_motion_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_panthera_motion_core HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core moveit_ros_planning rclcpp trajectory_msgs yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_catalog test/test_catalog.cpp)
  if(TARGET test_catalog)
    target_link_libraries(test_catalog panthera_motion_core)
    target_compile_definitions(
      test_catalog PRIVATE
      PANTHERA_TEST_CATALOG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_catalog.yaml"
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
