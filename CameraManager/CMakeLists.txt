cmake_minimum_required(VERSION 3.8)
project(camera_manager)

# TODO enable optimization on final builds
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE Release)
# endif()

# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraManager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

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
  src/utilities/commands/command_handler.cpp
  src/utilities/communication/streaming/streaming.cpp
  src/utilities/commands/command_board.cpp
  src/utilities/commands/command_generation.cpp
  src/utilities/communication/compression/H264Decoder.cpp
  src/utilities/communication/compression/H264Encoder.cpp
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIR}
  ${AVFORMAT_INCLUDE_DIR}
  ${AVUTIL_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(CameraManager
  ${OpenCV_LIBS}
  ${ZMQ_LIBRARIES}
  ${AVCODEC_LIBRARY}
  ${AVFORMAT_LIBRARY}
  ${AVUTIL_LIBRARY}
  ${Boost_LIBRARIES}
  -lboost_system
  -lpthread
  )

ament_target_dependencies(CameraManager
  rclcpp
  std_msgs
)

install(TARGETS
  CameraManager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
