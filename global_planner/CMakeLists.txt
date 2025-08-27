cmake_minimum_required(VERSION 3.5)

# Package name
project(global_planner)

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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(multirobot_utils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl 0.6 QUIET)

# Set the variable 'DEPENDENCIES' to the list of dependent packages
set(DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  multirobot_utils
)

# Add plugins (Internal Libraries)
file(GLOB PLUGIN_DIRS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/plugins" "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*")
set(PLUGIN_NAMES "")
foreach(PLUGIN_DIR ${PLUGIN_DIRS})
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/plugins/${PLUGIN_DIR}")
    list(APPEND PLUGIN_NAMES ${PLUGIN_DIR})
  endif()
endforeach()

set(PLUGIN_LIBRARIES "")

foreach(PLUGIN_NAME ${PLUGIN_NAMES})
  file(GLOB_RECURSE PLUGIN_SOURCES
    "plugins/${PLUGIN_NAME}/*.cpp"
  )

  add_library(${PLUGIN_NAME}
    SHARED
      ${PLUGIN_SOURCES}
  )

  target_include_directories(${PLUGIN_NAME}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/plugins>"
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/${PLUGIN_NAME}>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

  ament_target_dependencies(${PLUGIN_NAME}
    ${DEPENDENCIES})

  target_link_libraries(${PLUGIN_NAME}
    yaml-cpp
    Eigen3::Eigen
    fcl
  )

  list(APPEND PLUGIN_LIBRARIES ${PLUGIN_NAME})
endforeach()

# Add main library (Intenal Library)
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
  yaml-cpp
  Eigen3::Eigen
  fcl
  ${PLUGIN_LIBRARIES}
)

# Add main executable
add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
  ${LIBRARY_NAME}
  yaml-cpp
)

# Install
install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

foreach(PLUGIN_NAME ${PLUGIN_NAMES})
    install(TARGETS ${PLUGIN_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endforeach()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
