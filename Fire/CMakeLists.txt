cmake_minimum_required(VERSION 2.8)
project(su_v2)
find_package( OpenCV REQUIRED)
include_directories( ${openCV_INCLUDE_DIRS})
add_executable(su_v2 su_v2.cpp)
target_link_libraries(su_v2 ${OpenCV_LIBS})

