cmake_minimum_required(VERSION 3.5)
project(sigma)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Include directories
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(tf2_kdl REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


## Create executable
add_executable(master
    src/main_master.cpp
    src/SigmaDevice.cpp
    src/SigmaDevice.hpp
)

## Link libraries
ament_target_dependencies(master
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
   # tf2_kdl
    # tf2_geometry_msgs
)

target_link_libraries(master
    dhd
    drd
    usb-1.0
    pthread
    #tf2_kdl
)

## Install executable
install(TARGETS master
    DESTINATION lib/${PROJECT_NAME}
)

## Install headers
install(DIRECTORY include/
    DESTINATION include
)


install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
