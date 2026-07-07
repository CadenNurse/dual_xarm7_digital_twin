
cmake_minimum_required(VERSION 3.10)
project(trac_ik_kinematics_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

set(TRAC_IK_LIBRARY_NAME trac_ik_kinematics_plugin)

add_library(${TRAC_IK_LIBRARY_NAME} SHARED
  src/trac_ik_kinematics_plugin.cpp
)

target_include_directories(${TRAC_IK_LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${NLOPT_INCLUDE_DIRS}
)

target_link_libraries(${TRAC_IK_LIBRARY_NAME}
  ${NLOPT_LIBRARIES}
)

ament_target_dependencies(${TRAC_IK_LIBRARY_NAME}
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  trac_ik_lib
  Eigen3
)

pluginlib_export_plugin_description_file(moveit_core trac_ik_kinematics_description.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${TRAC_IK_LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  trac_ik_lib
  Eigen3
)

ament_package()