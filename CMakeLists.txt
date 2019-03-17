cmake_minimum_required(VERSION 3.13)
project(kdTree)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LZ4 REQUIRED liblz4)

find_package (PCL 1.8 REQUIRED)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)  #指定pcl1.8路径

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kdTree main.cpp)
target_link_libraries (kdTree ${PCL_LIBRARIES})