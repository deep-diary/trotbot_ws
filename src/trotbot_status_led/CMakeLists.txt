cmake_minimum_required(VERSION 3.16)
project(trotbot_status_led)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD libgpiod)

if(GPIOD_FOUND)
  message(STATUS "trotbot_status_led: libgpiod found, building hardware WS2812 driver")
  set(TSL_WSRC "src/ws2812_gpiod.cpp")
else()
  message(WARNING "trotbot_status_led: libgpiod NOT found — building stub GPIO driver (install libgpiod-dev on target)")
  set(TSL_WSRC "src/ws2812_gpiod_stub.cpp")
endif()

add_executable(status_led_node
  src/status_led_node.cpp
  src/ws2812_rockchip_mmap.cpp
  ${TSL_WSRC}
)
if(NOT GPIOD_FOUND)
  target_compile_definitions(status_led_node PRIVATE TROTBOT_STATUS_LED_GPIO_STUB=1)
endif()
target_include_directories(status_led_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(status_led_node
  rclcpp
  std_msgs
  ament_index_cpp
)
# 与 ament_target_dependencies 的 target_link 风格一致，使用非 keyword 形式
target_link_libraries(status_led_node yaml-cpp)
if(GPIOD_FOUND)
  target_compile_definitions(status_led_node PRIVATE TROTBOT_HAS_GPIOD_MUX=1)
  target_include_directories(status_led_node PRIVATE ${GPIOD_INCLUDE_DIRS})
  target_compile_options(status_led_node PRIVATE ${GPIOD_CFLAGS_OTHER})
  target_link_libraries(status_led_node ${GPIOD_LIBRARIES})
endif()

install(TARGETS status_led_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/test_status_led_manual.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
