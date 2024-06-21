cmake_minimum_required(VERSION 3.8)
project(server_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(messages REQUIRED)

include_directories(include)

add_executable(cloud src/cloud.cpp)
ament_target_dependencies(cloud rclcpp messages)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp rclcpp_lifecycle messages)

install(TARGETS
  cloud
  client
DESTINATION lib/${PROJECT_NAME})

ament_package()
