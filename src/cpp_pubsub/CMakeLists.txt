cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  talker
  src/publisher_member_function.cpp
)

ament_target_dependencies(
  talker
  rclcpp
  std_msgs
)

add_executable(
  listener
  src/subscriber_member_function.cpp
)

ament_target_dependencies(
  listener
  rclcpp
  std_msgs
)

add_executable(
  stm32_node
  src/stm32_node.cpp
)

ament_target_dependencies(
  stm32_node
  rclcpp
  std_msgs
)

add_executable(
  app_controller
  src/app_controller.cpp
)

ament_target_dependencies(
  app_controller
  rclcpp
  std_msgs
)

install(
  TARGETS
    talker
    listener
    stm32_node
    app_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
