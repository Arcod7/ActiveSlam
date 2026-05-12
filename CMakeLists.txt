cmake_minimum_required(VERSION 3.8)
project(basic_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 2. Déclaration de l'exécutable (AVANT les includes)
add_executable(basic_slam_node
  src/main.cpp
  src/basic_slam.cpp
)

# 3. Chemins d'inclusion pour la cible 'basic_slam_node'
target_include_directories(basic_slam_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. Liens vers les dépendances (Crucial pour que VSCode trouve les headers rclcpp, pcl, etc.)
ament_target_dependencies(basic_slam_node
  rclcpp

  std_msgs
  sensor_msgs
  nav_msgs
  
  pcl_conversions
  pcl_ros

  cv_bridge
  OpenCV
)

# 5. Installation
install(TARGETS
  basic_slam_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_to_tf)

install(PROGRAMS
  scripts/odom_to_tf_noisy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_to_tf_noisy)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
