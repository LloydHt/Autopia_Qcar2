cmake_minimum_required(VERSION 3.8)
project(qcar2_perception)

# C++17 obligatoire pour std::clamp
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# DÉPENDANCES
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)



# =========================================================
# EXÉCUTABLE — Lane Following Node
# =========================================================
add_executable(lane_following_node
  src/lane_following_node.cpp
)
ament_target_dependencies(lane_following_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
)
# =========================================================
# EXÉCUTABLE — Stop Sign Node
# =========================================================
add_executable(stop_sign_node
  src/stop_sign_node.cpp
)
ament_target_dependencies(stop_sign_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
)
# =========================================================
# EXÉCUTABLE — Lidar Safety Node 
# =========================================================
add_executable(lidar_safety_node
  src/lidar_safety_node.cpp
)
ament_target_dependencies(lidar_safety_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# =========================================================
# EXÉCUTABLE — cmd_vel_arbiter
# =========================================================

add_executable(cmd_vel_arbiter src/cmd_vel_arbiter.cpp)
ament_target_dependencies(cmd_vel_arbiter
  rclcpp geometry_msgs std_msgs
)

# Exécutable HD Map
add_executable(hdmap_navigation_node
  src/hdmap_navigation_node.cpp
)
ament_target_dependencies(hdmap_navigation_node
  rclcpp nav_msgs geometry_msgs std_msgs
)

target_include_directories(hdmap_navigation_node PUBLIC
  include
  map        
)

add_executable(ground_truth_publisher
  src/ground_truth_publisher.cpp
)
ament_target_dependencies(ground_truth_publisher
  rclcpp tf2_msgs nav_msgs geometry_msgs
)

# =========================================================
# INSTALLATION
# =========================================================
install(TARGETS
  lane_following_node
  lidar_safety_node
  stop_sign_node
  cmd_vel_arbiter
  hdmap_navigation_node
  ground_truth_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()