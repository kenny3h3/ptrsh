cmake_minimum_required(VERSION 3.16)
project(spot_micro_motion_cmd)

add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(i2c_pwm_board_msgs REQUIRED)
find_package(spot_micro_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(spot_micro_motion_cmd_node
  src/spot_micro_motion_cmd_node.cpp
  src/spot_micro_motion_cmd.cpp
)

target_include_directories(spot_micro_motion_cmd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(spot_micro_motion_cmd_node
  rclcpp
  std_msgs
  geometry_msgs
  i2c_pwm_board_msgs
  spot_micro_kinematics
)
target_link_libraries(spot_micro_motion_cmd_node Eigen3::Eigen)

install(TARGETS spot_micro_motion_cmd_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
