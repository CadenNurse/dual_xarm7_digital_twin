cmake_minimum_required(VERSION 3.16)
project(mtc_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

add_executable(minimal src/minimal.cpp)
ament_target_dependencies(minimal
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

add_executable(mtc_node_cntrl src/mtc_node_cntrl.cpp)
ament_target_dependencies(mtc_node_cntrl
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

install(TARGETS
  mtc_node
  mtc_node_cntrl
  minimal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()