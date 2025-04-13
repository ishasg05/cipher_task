cmake_minimum_required(VERSION 3.8)
project(caesar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cipher_interfaces REQUIRED)

add_executable(check_answer src/check_answer.cpp)
ament_target_dependencies(check_answer rclcpp cipher_interfaces)

add_executable(caesar_node src/cipher_node.cpp)
ament_target_dependencies(caesar_node rclcpp cipher_interfaces)

install(TARGETS
  caesar_node
  check_answer
  DESTINATION lib/${PROJECT_NAME})

ament_package()