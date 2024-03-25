cmake_minimum_required(VERSION 3.8)
project(mrobot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(
  set_init_amcl_pose src/set_init_amcl_pose.cpp
)
ament_target_dependencies(
  set_init_amcl_pose
  rclcpp geometry_msgs tf2
)

add_executable(
  move_mrobot src/move_mrobot.cpp
)
ament_target_dependencies(
  move_mrobot
  rclcpp geometry_msgs
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS
  set_init_amcl_pose
  move_mrobot
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
