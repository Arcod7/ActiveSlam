cmake_minimum_required(VERSION 3.8)
project(tsdf_octomap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(tsdf_to_octomap src/tsdf_to_octomap.cpp)
target_include_directories(tsdf_to_octomap PRIVATE ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(tsdf_to_octomap rclcpp sensor_msgs octomap_msgs)
target_link_libraries(tsdf_to_octomap ${OCTOMAP_LIBRARIES})

install(TARGETS tsdf_to_octomap DESTINATION lib/${PROJECT_NAME})

ament_package()
