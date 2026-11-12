cmake_minimum_required(VERSION 3.22)
project(agent_control_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# Zsibot SDK (prebuilt static library; same import as control_demo/read_state).
get_filename_component(_zsibot_default_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../zsibot-sdk/genisom_L1_sdk" ABSOLUTE)
set(ZSIBOT_SDK_DIR "${_zsibot_default_dir}"
  CACHE PATH "genisom_L1_sdk root directory")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(_zsibot_default_arch "aarch64")
else()
  set(_zsibot_default_arch "x86_64")
endif()
set(ZSIBOT_ARCH "${_zsibot_default_arch}"
  CACHE STRING "Zsibot prebuilt library architecture")

set(_zsibot_include_dir "${ZSIBOT_SDK_DIR}/include")
set(_zsibot_lib "${ZSIBOT_SDK_DIR}/lib/${ZSIBOT_ARCH}/libzsibot.a")

if(NOT EXISTS "${_zsibot_lib}")
  message(FATAL_ERROR "Zsibot static library not found: ${_zsibot_lib}")
endif()
if(NOT EXISTS "${_zsibot_include_dir}/zsibot_sdk/zsibot_api.h")
  message(FATAL_ERROR "Zsibot headers not found: ${_zsibot_include_dir}")
endif()

add_library(zsibot_sdk STATIC IMPORTED)
set_target_properties(zsibot_sdk PROPERTIES
  IMPORTED_LOCATION "${_zsibot_lib}"
  INTERFACE_INCLUDE_DIRECTORIES "${_zsibot_include_dir}"
  INTERFACE_LINK_LIBRARIES "Threads::Threads")

message(STATUS "agent_control_driver: zsibot_sdk = ${_zsibot_lib}")

add_executable(agent_control_driver
  src/control_node.cpp
  src/executor_holder.cpp
  src/motion_controller.cpp
  src/protocol.cpp
  src/json_util.cpp
  src/status_builder.cpp)

target_compile_features(agent_control_driver PRIVATE cxx_std_20)
target_include_directories(agent_control_driver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(agent_control_driver rclcpp std_msgs)
target_link_libraries(agent_control_driver zsibot_sdk)
target_compile_options(agent_control_driver PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sdk_axis_probe src/sdk_axis_probe.cpp)
target_compile_features(sdk_axis_probe PRIVATE cxx_std_20)
target_include_directories(sdk_axis_probe PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sdk_axis_probe zsibot_sdk)
target_compile_options(sdk_axis_probe PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS agent_control_driver sdk_axis_probe
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_axis_mapping test/test_axis_mapping.cpp)
  target_include_directories(test_axis_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME agent_control_axis_mapping COMMAND test_axis_mapping)

  add_executable(test_motion_controller
    test/test_motion_controller.cpp
    src/motion_controller.cpp)
  target_include_directories(test_motion_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME agent_control_motion_controller COMMAND test_motion_controller)
endif()

ament_package()
