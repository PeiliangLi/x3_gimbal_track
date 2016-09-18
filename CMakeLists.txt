cmake_minimum_required(VERSION 2.8.3)
project(gimbal_track)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    )

catkin_package()


include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_executable(gimbal_track
    src/gimbal_track_node.cpp
    )

target_link_libraries(gimbal_track ${catkin_LIBRARIES} )
