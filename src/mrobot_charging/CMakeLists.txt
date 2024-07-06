cmake_minimum_required(VERSION 3.8)
project(mrobot_charging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


include_directories(include)

# add_executable(
#   bt_ros_node 
#   src/bt_ros_node.cpp
#   src/charging_behaviors.cpp
#   src/marker_detector_node.cpp
# )
# ament_target_dependencies(
#   bt_ros_node
#   rclcpp behaviortree_cpp rclcpp_action nav2_msgs tf2 tf2_ros OpenCV cv_bridge geometry_msgs tf2_geometry_msgs mrobot_msgs
# )

add_executable(
  autonomous_charging_node
  src/autonomous_charging_node.cpp
)
ament_target_dependencies(
  autonomous_charging_node
  rclcpp tf2 tf2_ros geometry_msgs tf2_geometry_msgs nav_msgs
)

add_executable(
  marker_detector_node
  src/marker_detector_node.cpp
)
ament_target_dependencies(
  marker_detector_node
  rclcpp sensor_msgs tf2 tf2_ros OpenCV cv_bridge geometry_msgs tf2_geometry_msgs
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS
  autonomous_charging_node
  marker_detector_node
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
