cmake_minimum_required(VERSION 3.8)
project(mrs_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(serial_port src/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${mrs_lib_INCLUDE_DIRS})

ament_target_dependencies(serial_port
  rclcpp
  sensor_msgs
  std_srvs
  mrs_lib
  mrs_msgs
)

add_executable(vio_imu src/vio_imu_node.cpp src/vio_imu.cpp)
target_include_directories(vio_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${mrs_lib_INCLUDE_DIRS})
target_compile_features(vio_imu PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(vio_imu serial_port)

ament_target_dependencies(vio_imu
  rclcpp
  sensor_msgs
  std_srvs
  mrs_lib
  mrs_msgs
)

install(TARGETS vio_imu
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
