cmake_minimum_required(VERSION 3.1.2)
project(mrs_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  nodelet
  sensor_msgs
  mrs_msgs
  std_msgs
  mrs_lib
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
  config/gimbal.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BacaProtocol NmeaParser VioImu Gimbal
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs mrs_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
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

add_library(Led src/led.cpp src/serial_port.cpp)

target_link_libraries(Led
  ${catkin_LIBRARIES}
  )

add_library(Estop src/estop.cpp src/serial_port.cpp)

target_link_libraries(Estop
  ${catkin_LIBRARIES}
  )

add_library(Ultrasound src/ultrasound.cpp src/serial_port.cpp)

target_link_libraries(Ultrasound
  ${catkin_LIBRARIES}
  )

add_library(TarotGimbal src/tarot_gimbal.cpp src/serial_port.cpp)

target_link_libraries(TarotGimbal
  ${catkin_LIBRARIES}
  )

add_library(Gimbal
  src/gimbal.cpp src/serial_port.cpp src/SBGC_lib/SBGC_cmd_helpers.cpp include/gimbal.hpp
  )

add_dependencies(Gimbal
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(Gimbal
  ${catkin_LIBRARIES}
  )
