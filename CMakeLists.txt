cmake_minimum_required(VERSION 2.8.3)
project(mrs_serial)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  sensor_msgs
  mrs_msgs
  std_msgs
  )

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BacaProtocol NmeaParser VioImu
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs mrs_msgs
  )

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# Declare a C++ library
add_library(VioImu src/vio_imu.cpp src/serial_port.cpp)

target_link_libraries(VioImu
  ${catkin_LIBRARIES}
  )

# Declare a C++ library
add_library(NmeaParser src/nmea_parser.cpp src/serial_port.cpp)

target_link_libraries(NmeaParser
  ${catkin_LIBRARIES}
  )

# Declare a C++ library
add_library(BacaProtocol src/baca_protocol.cpp src/serial_port.cpp)

target_link_libraries(BacaProtocol
  ${catkin_LIBRARIES}
  )
