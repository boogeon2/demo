cmake_minimum_required(VERSION 3.5)

# Package name
project(multirobot_utils)

# Default Settings for Cmake
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

add_definitions(-DMULTIROBOT_UTILS__USING_FCL_0_6)

# Find Dependent External Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl QUIET)

# Set the variable 'DEPENDENCIES' to the list of dependent packages
set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Add main library (Intenal Library)
set(LIBRARY_NAME
  ${PROJECT_NAME}
)

file(GLOB LIB_SOURCES
  src/*.cpp
  src/geometry/*.cpp
)

add_library(${PROJECT_NAME}
  SHARED
    ${LIB_SOURCES}
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${LIBRARY_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${LIBRARY_NAME}
  ${DEPENDENCIES})

# Install with export (For other packages to use)
install(TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  DESTINATION lib
)

ament_export_targets(export_${LIBRARY_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
