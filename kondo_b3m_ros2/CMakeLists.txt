cmake_minimum_required(VERSION 3.8)
project(kondo_b3m_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kondo_b3m_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(kondo_b3m
  src/kondo_b3m.cpp
  src/b3m_port.cpp
  src/b3m_motor.cpp
  src/b3m_command.cpp)
ament_target_dependencies(kondo_b3m rclcpp kondo_b3m_interfaces sensor_msgs)

install(TARGETS
  kondo_b3m
  DESTINATION lib/${PROJECT_NAME})

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
