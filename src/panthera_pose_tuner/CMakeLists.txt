cmake_minimum_required(VERSION 3.8)
project(panthera_pose_tuner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(panthera_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(pose_tuner_node
  src/TuningTargets.cpp
  src/pose_tuner_node.cpp
)

target_include_directories(pose_tuner_node
  PRIVATE
    include
)

ament_target_dependencies(pose_tuner_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  panthera_interfaces
  std_srvs
)

install(TARGETS
  pose_tuner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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
