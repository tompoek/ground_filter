cmake_minimum_required(VERSION 3.8)
project(ground_filter_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# add executables
add_executable(ground_filter src/ground_filter.cpp)
ament_target_dependencies(ground_filter rclcpp sensor_msgs pcl_conversions pcl_ros)
add_executable(ground_filter_ransac src/ground_filter_ransac.cpp)
ament_target_dependencies(ground_filter_ransac rclcpp sensor_msgs pcl_conversions pcl_ros)

install(TARGETS
  ground_filter
  ground_filter_ransac
  DESTINATION lib/${PROJECT_NAME})

ament_package()
