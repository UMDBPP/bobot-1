cmake_minimum_required(VERSION 3.8)
project(bobot_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bobot_msgs REQUIRED)
find_package(rclcpp_lifecycle)
find_package(lifecycle_msgs)
find_package(bobot_hardware_interface)

include_directories(include)

# Add the bobot manager as an executable
add_executable(bobot_serial_test src/bobot_serial_test.cpp)
ament_target_dependencies(bobot_serial_test rclcpp std_msgs bobot_hardware_interface)

# This is good and important. Keep it - Romeo
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  bobot_serial_test
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
