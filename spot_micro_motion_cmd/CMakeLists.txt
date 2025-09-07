cmake_minimum_required(VERSION 3.16)
project(spot_micro_motion_cmd)

# Compiler-Flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++14)
endif()

# --- ROS 2 / Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Die Kinematics kommt als separates ROS-2-Paket
find_package(spot_micro_kinematics REQUIRED)

# i2c_pwm_board wird nur zur Laufzeit benötigt (wir includen keine Header daraus)
# -> kein find_package nötig, nur exec_depend in package.xml

# --- Libraries aus diesem Paket ---
add_library(rate_limited_filter src/rate_limited_first_order_filter.cpp)
target_include_directories(rate_limited_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/spot_micro_motion_cmd>
  $<INSTALL_INTERFACE:include/spot_micro_motion_cmd>
)

add_library(smfsm
  src/spot_micro_state.cpp
  src/spot_micro_idle.cpp
  src/spot_micro_stand.cpp
  src/spot_micro_transition_stand.cpp
  src/spot_micro_transition_idle.cpp
  src/spot_micro_walk.cpp
)
target_include_directories(smfsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/spot_micro_motion_cmd>
  $<INSTALL_INTERFACE:include/spot_micro_motion_cmd>
)
target_link_libraries(smfsm
  spot_micro_kinematics         # externe Kinematics-Lib
  Eigen3::Eigen
)

# --- Executable ---
add_executable(spot_micro_motion_cmd_node
  src/spot_micro_motion_cmd.cpp
  src/spot_micro_motion_cmd_node.cpp
  src/utils.cpp
)
target_include_directories(spot_micro_motion_cmd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/spot_micro_motion_cmd>
  $<INSTALL_INTERFACE:include/spot_micro_motion_cmd>
)
target_link_libraries(spot_micro_motion_cmd_node
  smfsm
  rate_limited_filter
  spot_micro_kinematics
  Eigen3::Eigen
)
ament_target_dependencies(spot_micro_motion_cmd_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- Install ---
install(TARGETS
  rate_limited_filter
  smfsm
  spot_micro_motion_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
