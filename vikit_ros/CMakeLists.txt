cmake_minimum_required(VERSION 3.8)
project(vikit_ros)

# Default to Release build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C++ Standard (ROS 2 Humble+ typically uses C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler Flags
add_compile_options(-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vikit_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

# Include Directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# Source Files
set(SOURCEFILES 
  src/output_helper.cpp
)

# Library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  vikit_common
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  OpenCV
  Sophus
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES} 
)

# Install Rules
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export Dependencies for downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  vikit_common
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  OpenCV
  Sophus
)

ament_package()