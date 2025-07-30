cmake_minimum_required(VERSION 3.8)
project(collatz_cpp_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(collatz_core REQUIRED)

add_executable(cpp_server src/cpp_server.cpp)
ament_target_dependencies(cpp_server rclcpp collatz_core) 

install(TARGETS
  cpp_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
