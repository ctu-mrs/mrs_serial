cmake_minimum_required(VERSION 3.1.2)
project(mrs_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  sensor_msgs
  mrs_msgs
  libmavconn
  std_msgs
  mrs_lib
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BacaProtocol NmeaParser VioImu Gimbal
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs mrs_msgs libmavconn
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(VioImu src/vio_imu.cpp src/serial_port.cpp)
target_link_libraries(VioImu
  ${catkin_LIBRARIES}
  )

add_library(NmeaParser src/nmea_parser.cpp src/serial_port.cpp)
target_link_libraries(NmeaParser
  ${catkin_LIBRARIES}
  )

add_library(BacaProtocol src/baca_protocol.cpp src/serial_port.cpp)
target_link_libraries(BacaProtocol
  ${catkin_LIBRARIES}
  )

add_library(Servo src/servo.cpp src/serial_port.cpp)
target_link_libraries(Servo
  ${catkin_LIBRARIES}
  )

add_library(Gimbal src/gimbal_mavlink.cpp)
target_link_libraries(Gimbal
  ${catkin_LIBRARIES}
  )
