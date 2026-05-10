cmake_minimum_required(VERSION 3.10)

project(icar)

option(ENABLE_X86_MOCK "Enable x86 mock camera/uart" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})

if(NOT ENABLE_X86_MOCK)
    pkg_search_module(SERIAL REQUIRED libserial)
    include_directories(${SERIAL_INCLUDE_DIRS})
    link_directories(${SERIAL_LIBRARY_DIRS})
endif()

add_executable(icar main.cpp
    param/param.hpp
    capture/capture.h
    capture/capture.cpp
    thread/thread.h
    thread/thread.cpp
    include/common.hpp
    include/detection.hpp
    include/json.hpp
    include/uart.hpp
    include/hw_interface.hpp
    hardware/hw_factory.cpp
    include/logger.hpp
    track/imgprocess/imgprocess.h
    track/imgprocess/imgprocess.cpp
    track/standard/standard.h
    track/standard/standard.cpp
    track/standard/general.h
    track/basic/ring.cpp
    track/basic/ring.h
    track/basic/cross.cpp
    track/basic/cross.h
    track/special/catering.h
    track/special/catering.cpp
    track/special/obstacle.h
    track/special/obstacle.cpp
    track/special/layby.h
    track/special/layby.cpp
    track/special/bridge.h
    track/special/bridge.cpp
    track/special/charging.h
    track/special/charging.cpp
    track/special/crosswalk.h
    track/special/crosswalk.cpp
)

if(ENABLE_X86_MOCK)
    target_compile_definitions(icar PRIVATE ENABLE_X86_MOCK)
endif()

target_link_libraries(icar
    ${OpenCV_LIBRARIES}
    ${PPNC_LIBRARIES}
    ${ONNX_LIBRARIES}
    pthread
)

if(NOT ENABLE_X86_MOCK)
    target_link_libraries(icar ${SERIAL_LDFLAGS})
endif()
