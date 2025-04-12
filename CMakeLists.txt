cmake_minimum_required(VERSION 3.8)
project(camera_manager)

# [ ] enable optimization on final builds
# [ ] also may wanna make this unconditional, idk what colcon does
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE Release)
# endif()

# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/websocketpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraManager
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(robot_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(CameraManager 
  src/CameraManager.cpp
  src/utilities/cameras/startup.cpp
  src/utilities/cameras/camera_object.cpp
  src/utilities/cameras/settings.cpp
  src/utilities/cameras/camera_thread.cpp
  src/utilities/cameras/realsense_interface.cpp
  src/utilities/commands/command_handler.cpp
  src/utilities/communication/streaming/streaming.cpp
  src/utilities/commands/command_board.cpp
  src/utilities/commands/command_generation.cpp
  src/utilities/communication/compression/Encoder.cpp
  src/utilities/communication/compression/Decoder.cpp
)
find_package(OpenSSL REQUIRED)
# FIXME remove
target_link_libraries(CameraManager OpenSSL::SSL OpenSSL::Crypto)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(CameraManager
  ${OpenCV_LIBS}
)

ament_target_dependencies(CameraManager
  rclcpp
  std_msgs
  sensor_msgs
  robot_interfaces
)

# Executable install
install(TARGETS
  CameraManager
  DESTINATION lib/${PROJECT_NAME}
)

# Launch install
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
