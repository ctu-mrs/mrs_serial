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

set(LIBRARIES
  VioImu NmeaParser BacaProtocol Servo Led Estop Ultrasound TarotGimbal Gimbal
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs mrs_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Eagle

add_library(Eagle
  src/eagle.cpp
  src/serial_port.cpp
  )

# VioImu

add_library(VioImu
  src/vio_imu.cpp
  src/serial_port.cpp
  )

target_link_libraries(VioImu
  ${catkin_LIBRARIES}
  )

# NmeaParser

add_library(NmeaParser
  src/nmea_parser.cpp
  src/serial_port.cpp
  )

target_link_libraries(NmeaParser
  ${catkin_LIBRARIES}
  )

# BacaProtocol

add_library(BacaProtocol
  src/baca_protocol.cpp
  src/serial_port.cpp
  )

target_link_libraries(BacaProtocol
  ${catkin_LIBRARIES}
  )

# Servo

add_library(Servo
  src/servo.cpp
  src/serial_port.cpp
  )

target_link_libraries(Servo
  ${catkin_LIBRARIES}
  )

# Led

add_library(Led
  src/led.cpp
  src/serial_port.cpp
  )

target_link_libraries(Led
  ${catkin_LIBRARIES}
  )

# Estop

add_library(Estop
  src/estop.cpp
  src/serial_port.cpp
  )

target_link_libraries(Estop
  ${catkin_LIBRARIES}
  )

# Ultrasound

add_library(Ultrasound
  src/ultrasound.cpp
  src/serial_port.cpp)

target_link_libraries(Ultrasound
  ${catkin_LIBRARIES}
  )

# TarotGimbal

add_library(TarotGimbal
  src/tarot_gimbal.cpp
  src/serial_port.cpp)

target_link_libraries(TarotGimbal
  ${catkin_LIBRARIES}
  )

# Gimbal

add_library(Gimbal
  src/gimbal.cpp
  src/serial_port.cpp
  src/SBGC_lib/SBGC_cmd_helpers.cpp
  include/gimbal.hpp
  )

add_dependencies(Gimbal
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(Gimbal
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux_scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
