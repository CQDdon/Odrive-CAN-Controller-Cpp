cmake_minimum_required(VERSION 3.8)
project(odrive_can_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#Publisher
add_executable(keyboard_sender_cpp_node src/keyboard_sender.cpp)
ament_target_dependencies(keyboard_sender_cpp_node rclcpp std_msgs)

#Subsciber
add_executable(odrive_sender_cpp_node src/odrive_sender.cpp)
ament_target_dependencies(odrive_sender_cpp_node rclcpp std_msgs)

install(TARGETS
  #don't forget to change the node's name here
  keyboard_sender_cpp_node
  odrive_sender_cpp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
