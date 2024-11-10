cmake_minimum_required(VERSION 3.5.0)
project(CameraManager VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(CameraManager 
CameraManager.cpp
utilities/cameras/startup.cpp
utilities/cameras/camera_object.cpp
utilities/cameras/settings.cpp
utilities/commands/command_handler.cpp
utilities/communication/streaming/streaming.cpp
utilities/commands/command_board.cpp
utilities/communication/compression/H264Decoder.cpp
utilities/communication/compression/H264Encoder.cpp
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

include_directories(${OpenCV_INCLUDE_DIRS}
${AVCODEC_INCLUDE_DIR}
${AVFORMAT_INCLUDE_DIR}
${AVUTIL_INCLUDE_DIR}
${Boost_INCLUDE_DIRS})

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
