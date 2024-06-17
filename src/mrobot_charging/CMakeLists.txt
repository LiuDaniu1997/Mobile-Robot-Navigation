cmake_minimum_required(VERSION 3.8)
project(mrobot_charging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

add_executable(
  bt_ros_node 
  src/bt_ros_node.cpp
  src/charging_behaviors.cpp
)
ament_target_dependencies(
  bt_ros_node
  rclcpp behaviortree_cpp
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS
  bt_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
