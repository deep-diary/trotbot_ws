cmake_minimum_required(VERSION 3.5)
project(trotbot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python package
ament_python_install_package(hal)

# Install Python executables
install(PROGRAMS
  hal/interfaces/servo_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_interface
)

install(PROGRAMS
  hal/utils/calibrate_servos.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrate_servos
)

install(PROGRAMS
  hal/utils/calibrate_tool.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrate_tool
)

install(PROGRAMS
  hal/utils/calibration_status.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibration_status
)

# Install data directories
install(DIRECTORY
  launch
  config
  urdf
  meshes
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install resource marker
install(FILES
  resource/trotbot
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
