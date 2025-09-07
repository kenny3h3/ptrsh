cmake_minimum_required(VERSION 3.16)
project(spot_micro_motion_cmd)

# Compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(i2c_pwm_board_msgs REQUIRED)
find_package(spot_micro_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Executable: alle vorhandenen Quellen ----
add_executable(spot_micro_motion_cmd_node
  src/spot_micro_motion_cmd_node.cpp
  src/spot_micro_motion_cmd.cpp
  src/spot_micro_state.cpp
  src/spot_micro_idle.cpp
  src/spot_micro_stand.cpp
  src/spot_micro_transition_idle.cpp
  src/spot_micro_transition_stand.cpp
  src/spot_micro_walk.cpp
  src/utils.cpp
  # rate_limited_first_order_filter.h ist header-only → KEINE .cpp hier eintragen
)

# Includes (Headers liegen bei dir im Paket-Root bzw. include/, passe bei Bedarf an)
target_include_directories(spot_micro_motion_cmd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Link- und Paketabhängigkeiten
ament_target_dependencies(spot_micro_motion_cmd_node
  rclcpp
  std_msgs
  geometry_msgs
  i2c_pwm_board_msgs
  spot_micro_kinematics
)

target_link_libraries(spot_micro_motion_cmd_node
  Eigen3::Eigen
)

# ---- Install ----
install(TARGETS spot_micro_motion_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
