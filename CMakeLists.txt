cmake_minimum_required(VERSION 3.9)
project(virtual_lidar_bumper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)

add_executable(virtual_lidar_bumper_node src/virtual_lidar_bumper_node.cpp)
target_include_directories(virtual_lidar_bumper_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(virtual_lidar_bumper_node
  ${dependencies}
)

target_link_libraries(virtual_lidar_bumper_node
  ${PCL_LIBRARIES}
)

install(TARGETS virtual_lidar_bumper_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
