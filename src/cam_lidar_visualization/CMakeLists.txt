cmake_minimum_required(VERSION 3.0.2)
project(cam_lidar_visualization)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(ATTRIBUTE_FLAG "-Wignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ATTRIBUTE_FLAG}")

find_package(OpenCV REQUIRED)
find_package(OpenCL REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # OpenCV
  ${OpenCV_INCLUDE_DIRS}
  # OpenCL
  ${OpenCL_INCLUDE_DIR}
  ${OpenCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
 src/cam_lidar_visualization.cpp
)

add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    # OpenCV
    ${OpenCV_LIBRARIES}
    # OpenCL
    ${OpenCL_LIBRARY}
    ${PCL_LIBRARIES}
)

