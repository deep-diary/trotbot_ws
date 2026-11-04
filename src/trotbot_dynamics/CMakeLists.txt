cmake_minimum_required(VERSION 3.8)
project(trotbot_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(trotbot_can_bridge REQUIRED)

add_executable(pinocchio_tau_ff_node
  src/pinocchio_tau_ff_node.cpp
)
ament_target_dependencies(pinocchio_tau_ff_node
  rclcpp
  sensor_msgs
  std_msgs
  trotbot_can_bridge
)
target_link_libraries(pinocchio_tau_ff_node pinocchio::pinocchio)

install(TARGETS
  pinocchio_tau_ff_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
