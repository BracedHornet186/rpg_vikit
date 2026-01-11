cmake_minimum_required(VERSION 3.8)
project(vikit_common)

# --- Options and Build Type ---
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Build with ROS 2 (ament) by default. 
# Set -DUSE_ROS=OFF to build as a pure CMake package.
option(USE_ROS "Build with ROS 2 ament dependencies" ON)

# --- Compiler Flags & Architecture ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3")
else()
  # ARM optimization
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()

# Release optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# --- Dependencies ---
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

if(USE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED) 
endif()

# --- Include Directories ---
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# --- Sources ---
set(SOURCEFILES 
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/equidistant_camera.cpp
  src/polynomial_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

# --- Library ---
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES} 
)

if(USE_ROS)
  ament_target_dependencies(${PROJECT_NAME} rclcpp)
endif()

# --- Tests ---
# Note: In ROS 2, it is preferred to use ament_add_test, but preserving original structure here.
if(BUILD_TESTING)
  add_executable(test_vk_common_camera test/test_camera.cpp)
  target_link_libraries(test_vk_common_camera ${PROJECT_NAME})

  add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
  target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME})

  add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
  target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME})
endif()

# --- Installation & Export ---
if(USE_ROS)
  # Install the main library
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # Install headers
  install(
    DIRECTORY include/
    DESTINATION include
  )

  # Export dependencies for downstream packages (e.g. vikit_ros)
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(Eigen3 OpenCV Sophus)
  
  if(USE_ROS)
    ament_export_dependencies(rclcpp)
  endif()

  ament_package()
else()
  # Pure CMake install logic (Legacy support)
  install(DIRECTORY include/vikit DESTINATION include FILES_MATCHING PATTERN "*.h" )
  install(TARGETS ${PROJECT_NAME} DESTINATION lib)
endif()