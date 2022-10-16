cmake_minimum_required(VERSION 3.5)
project(kondo_b3m_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/DesiredPosition.msg"
  "msg/DesiredSpeed.msg"
)
set(srv_files
  "srv/DesiredPosition.srv"
  "srv/DesiredSpeed.srv"
  "srv/MotorFree.srv"
  "srv/StartPositionControl.srv"
  "srv/StartSpeedControl.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(kondo_b3m
  src/kondo_b3m.cpp
  src/b3m_port.cpp
)
ament_target_dependencies(kondo_b3m
  rclcpp
  sensor_msgs
)
target_include_directories(kondo_b3m
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rosidl_target_interfaces(kondo_b3m
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(util_id_changer
  src/util_id_changer.cpp
  src/b3m_port.cpp
)
ament_target_dependencies(util_id_changer
  rclcpp
)
target_include_directories(util_id_changer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  kondo_b3m
  util_id_changer
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
