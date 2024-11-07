cmake_minimum_required(VERSION 3.5.0)
project(CameraManager VERSION 0.1.0 LANGUAGES C CXX)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(CameraManager 
CameraManager.cpp
utilities/cameras/startup.cpp
utilities/cameras/camera_object.cpp
utilities/cameras/settings.cpp
utilities/commands/command_handler.cpp
utilities/communication/streaming/streaming.cpp
utilities/commands/command_board.cpp)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(CameraManager
${OpenCV_LIBS}
${ZMQ_LIBRARIES})
