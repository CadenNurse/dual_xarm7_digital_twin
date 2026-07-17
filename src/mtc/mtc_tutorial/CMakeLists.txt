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

add_executable(mtc_node_pp src/mtc_node_pp.cpp)
ament_target_dependencies(mtc_node_pp
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

add_executable(mtc_node_phoff src/mtc_node_phoff.cpp)
ament_target_dependencies(mtc_node_phoff
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

add_executable(mtc_node_shoff src/mtc_node_shoff.cpp)
ament_target_dependencies(mtc_node_shoff
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

add_executable(close_laptop src/close_laptop.cpp)
ament_target_dependencies(close_laptop
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

add_executable(laptop_hinge_state_publisher src/laptop_hinge_state_publisher.cpp)
ament_target_dependencies(laptop_hinge_state_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pick_place_eff src/pick_place_eff.cpp)
ament_target_dependencies(pick_place_eff
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
)

install(TARGETS
  mtc_node
  mtc_node_pp
  mtc_node_cntrl
  mtc_node_phoff
  mtc_node_shoff
  pick_place_eff
  close_laptop
  laptop_hinge_state_publisher
  minimal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()