cmake_minimum_required(VERSION 3.5)

# Package name
project(local_planner)

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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_interfaces REQUIRED)
find_package(multirobot_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl 0.6 QUIET)

# Set the variable 'DEPENDENCIES' to the list of dependent packages
set(DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  ros2_interfaces
  multirobot_utils
)

# Add main library (Internal Library)
set(LIBRARY_NAME
  ${PROJECT_NAME}_core
)

add_library(${LIBRARY_NAME}
  SHARED
    src/${PROJECT_NAME}.cpp
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${LIBRARY_NAME}
  ${DEPENDENCIES})

target_link_libraries(${LIBRARY_NAME}
  Eigen3::Eigen
  fcl
)

# Add main executable for real robot
add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME}
  ${LIBRARY_NAME}
)

# Install
install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
