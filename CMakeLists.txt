cmake_minimum_required(VERSION 3.5)
project(mrs_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  roscpp
  cmake_modules
  nodelet
  sensor_msgs
  mrs_msgs
  mrs_modules_msgs
  std_msgs
  mrs_lib
  dynamic_reconfigure
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  config/gimbal.cfg
  )

set(LIBRARIES
  MrsSerial_VioImu
  MrsSerial_NmeaParser
  MrsSerial_BacaProtocol
  MrsSerial_Servo
  MrsSerial_Led
  MrsSerial_Estop
  MrsSerial_Ultrasound
  MrsSerial_TarotGimbal
  MrsSerial_Gimbal
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Eagle

add_library(MrsSerial_Eagle
  src/eagle.cpp
  src/serial_port.cpp
  )

# MrsSerial_VioImu

add_library(MrsSerial_VioImu
  src/vio_imu.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsSerial_VioImu
  ${catkin_LIBRARIES}
  )

# MrsSerial_NmeaParser

add_library(MrsSerial_NmeaParser
  src/nmea_parser.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsSerial_NmeaParser
  ${catkin_LIBRARIES}
  )

# MrsSerial_BacaProtocol

add_library(MrsSerial_BacaProtocol
  src/baca_protocol.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsSerial_BacaProtocol
  ${catkin_LIBRARIES}
  )

# MrsSerial_Servo

add_library(MrsSerial_Servo
  src/servo.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsSerial_Servo
  ${catkin_LIBRARIES}
  )

# MrsSerial_Led

add_library(MrsSerial_Led
  src/led.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsSerial_Led
  ${catkin_LIBRARIES}
  )

# MrsSerial_Estop

add_library(MrsSerial_Estop
  src/estop.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsSerial_Estop
  ${catkin_LIBRARIES}
  )

# MrsSerial_Ultrasound

add_library(MrsSerial_Ultrasound
  src/ultrasound.cpp
  src/serial_port.cpp)

target_link_libraries(MrsSerial_Ultrasound
  ${catkin_LIBRARIES}
  )

# MrsSerial_TarotGimbal

add_library(MrsSerial_TarotGimbal
  src/tarot_gimbal.cpp
  src/serial_port.cpp)

target_link_libraries(MrsSerial_TarotGimbal
  ${catkin_LIBRARIES}
  )

# MrsSerial_Gimbal

add_library(MrsSerial_Gimbal
  src/gimbal.cpp
  src/serial_port.cpp
  src/SBGC_lib/SBGC_cmd_helpers.cpp
  include/gimbal.hpp
  )

add_dependencies(MrsSerial_Gimbal
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(MrsSerial_Gimbal
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

install(DIRECTORY include
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
