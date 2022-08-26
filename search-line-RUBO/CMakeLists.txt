cmake_minimum_required(VERSION 3.21)
project(untitled)

set(CMAKE_CXX_STANDARD 14)
add_executable(untitled main.cpp headfile.h zf_common_headfile.h element.cpp element.h cam.cpp cam.h image_deal.cpp image_deal.h All_Init.h All_Init.cpp ty.h ty.cpp control.cpp control.h Device.cpp Device.h elc.cpp elc.h zf_device_icm20602.h zf_device_icm20602.cpp zf_device_mt9v03x.cpp zf_device_mt9v03x.h RUBO_stack.h RUBO_stack.c RUBO_stack.cpp)
set(CMAKE_EXE_LINKER_FLAGS -static)
set(OpenCV_DIR C:\\OPENCV\\make)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled ${OpenCV_LIBS})

