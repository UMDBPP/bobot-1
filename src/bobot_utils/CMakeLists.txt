cmake_minimum_required(VERSION 3.8)
project(bobot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bobot_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_executable(timer src/timer_node.cpp)
add_executable(simple_timer src/simple_timer_node.cpp)
add_executable(servo_jerk src/servo_jerker.cpp)
add_executable(altitude_monitor src/altitude_monitor.cpp)

ament_target_dependencies(timer rclcpp std_msgs bobot_msgs)
ament_target_dependencies(simple_timer rclcpp std_msgs std_srvs)
ament_target_dependencies(servo_jerk rclcpp std_msgs bobot_msgs)
ament_target_dependencies(altitude_monitor rclcpp std_msgs bobot_msgs)

target_link_libraries(timer
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

target_link_libraries(servo_jerk
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

target_link_libraries(altitude_monitor
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
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

install(TARGETS
  timer
  simple_timer
  servo_jerk
  altitude_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
