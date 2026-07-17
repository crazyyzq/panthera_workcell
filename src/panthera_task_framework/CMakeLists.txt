cmake_minimum_required(VERSION 3.8)
project(panthera_task_framework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(panthera_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(workflow_executor_node
  src/workflow_executor_node.cpp
)

add_executable(pose_commissioning_node
  src/pose_commissioning_node.cpp
)

ament_target_dependencies(workflow_executor_node
  ament_index_cpp
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  moveit_ros_planning_interface
  panthera_interfaces
)

ament_target_dependencies(pose_commissioning_node
  ament_index_cpp
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(workflow_executor_node yaml-cpp::yaml-cpp)
  target_link_libraries(pose_commissioning_node yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(workflow_executor_node yaml-cpp)
  target_link_libraries(pose_commissioning_node yaml-cpp)
else()
  target_link_libraries(workflow_executor_node ${YAML_CPP_LIBRARIES})
  target_link_libraries(pose_commissioning_node ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS
  workflow_executor_node
  pose_commissioning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
