cmake_minimum_required(VERSION 3.8)
project(mrs_serial)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  ament_cmake
  sensor_msgs
  std_srvs
  mrs_lib
  mrs_msgs
  rcl_interfaces
  )

set(LIBRARIES
  MrsSerial_SerialPort
  MrsSerial_VioImu_component
  )

#set(EXECUTABLES
#  MrsSerial_VioImu
#  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${mrs_lib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

## | ----------------------- Serial Port ---------------------- |

add_library(MrsSerial_SerialPort SHARED
  src/serial_port.cpp
  )

ament_target_dependencies(MrsSerial_SerialPort
  ${DEPENDENCIES}
)

## | ------------------------- VIO IMU ------------------------ |

add_library(MrsSerial_VioImu_component SHARED
  #src/vio_imu_node.cpp
  src/vio_imu.cpp
)

target_compile_features(MrsSerial_VioImu_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(MrsSerial_VioImu_component
  MrsSerial_SerialPort
)

rclcpp_components_register_node(
    MrsSerial_VioImu_component
    PLUGIN "vio_imu::VioImu"
    EXECUTABLE MrsSerial_VioImu
)

ament_target_dependencies(MrsSerial_VioImu_component
  ${DEPENDENCIES}
)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
  )

install(TARGETS
  ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include
  DESTINATION .
  )

#install(TARGETS ${EXECUTABLES}
#  DESTINATION lib/${PROJECT_NAME}
#  )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
  )

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  )

ament_export_dependencies(
  ${DEPENDENCIES}
  )

ament_package()
