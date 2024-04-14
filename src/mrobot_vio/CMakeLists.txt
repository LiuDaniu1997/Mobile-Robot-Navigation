cmake_minimum_required(VERSION 3.8)
project(mrobot_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(
  feature_tracker 
  src/feature_tracker/feature_tracker.cpp
  src/feature_tracker/feature_tracker_node.cpp
  src/camera_model/pinhole_camera.cpp
)
ament_target_dependencies(
  feature_tracker
  rclcpp sensor_msgs cv_bridge OpenCV ${Eigen_LIBRARIES}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS
  feature_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
