cmake_minimum_required(VERSION 3.8)
project(spot_micro_motion_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros2_i2cpwmboard REQUIRED)

# Subdirectory für Kinematics-Lib
add_subdirectory(libs/spot_micro_kinematics_cpp)

# State Machine Lib
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
  libs/spot_micro_kinematics_cpp/include
)
target_link_libraries(smfsm spot_micro_kinematics Eigen3::Eigen)

# Filter Lib
add_library(rate_limited_filter src/rate_limited_first_order_filter.cpp)
target_include_directories(rate_limited_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/spot_micro_motion_cmd>
  $<INSTALL_INTERFACE:include/spot_micro_motion_cmd>
)

# Executable
add_executable(spot_micro_motion_cmd_node
  src/spot_micro_motion_cmd.cpp
  src/spot_micro_motion_cmd_node.cpp
  src/utils.cpp
)
target_include_directories(spot_micro_motion_cmd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/spot_micro_motion_cmd>
  $<INSTALL_INTERFACE:include/spot_micro_motion_cmd>
  libs/spot_micro_kinematics_cpp/include
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
  ros2_i2cpwmboard
)
rosidl_target_interfaces(spot_micro_motion_cmd_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  ros2_i2cpwmboard "rosidl_typesupport_cpp"
)

install(TARGETS spot_micro_motion_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch include libs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
