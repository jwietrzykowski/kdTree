cmake_minimum_required(VERSION 3.13)
project(KDT)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS False)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)

find_package (PCL 1.8 REQUIRED)
# TODO Ta zmienna powinna byc znajdowana automatycznie przez makro powyzej
#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(KDT main.cpp)  

target_link_libraries (KDT ${PCL_LIBRARIES})
