cmake_minimum_required(VERSION 3.8)
project(bobot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Inlude the "include/" directory of the package
include_directories(include)

add_library(bobot_hardware_interface-bobot_servo_interface SHARED src/bobot_servo_interface.cpp)
ament_target_dependencies( # Add whatever package dependencies this file needs (anything from the find package section)
  bobot_hardware_interface-bobot_servo_interface
  hardware_interface
  rclcpp
)

add_library(bobot_hardware_interface-bobot_altimeter_interface SHARED src/bobot_altimeter_interface.cpp)
ament_target_dependencies( # Add whatever package dependencies this file needs (anything from the find package section)
  bobot_hardware_interface-bobot_altimeter_interface
  hardware_interface
  rclcpp
)

add_library(bobot_hardware_interface-bobot_motor_interface SHARED src/bobot_motor_interface.cpp)
ament_target_dependencies( # Add whatever package dependencies this file needs (anything from the find package section)
  bobot_hardware_interface-bobot_motor_interface
  hardware_interface
  rclcpp
)

# Install all of the files in our include/ directory to "include"
install(
  DIRECTORY include/
  DESTINATION include
)

# Install our libraries (For each "add_library" call above, there should be a representative name under "TARGETS")
install(
  TARGETS
    bobot_hardware_interface-bobot_servo_interface
    bobot_hardware_interface-bobot_motor_interface
    bobot_hardware_interface-bobot_altimeter_interface
  EXPORT bobot_hardware_interfaceTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Do silly ament stuff (look it up or wait for me to add it later - Romeo)
ament_export_include_directories(include)
ament_export_targets(bobot_hardware_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  rclcpp
)
ament_package()

