cmake_minimum_required(VERSION 3.8)
project(logging_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(log_tester src/log_testing.cpp)
ament_target_dependencies(log_tester rclcpp)

install(TARGETS
  log_tester
  DESTINATION lib/${PROJECT_NAME})

ament_package()
