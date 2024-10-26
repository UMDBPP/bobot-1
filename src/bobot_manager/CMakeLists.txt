cmake_minimum_required(VERSION 3.8)
project(bobot_manager)

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
find_package(std_srvs REQUIRED)
find_package(bobot_msgs REQUIRED)
find_package(rclcpp_lifecycle)
find_package(lifecycle_msgs)

include_directories(include)

# Add the bobot manager as an executable
add_executable(manager src/bobot_manager.cpp)
ament_target_dependencies(manager rclcpp std_msgs std_srvs bobot_msgs lifecycle_msgs)

# This is good and important. Keep it - Romeo
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  manager
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
