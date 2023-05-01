cmake_minimum_required(VERSION 3.0.2)
project(depth_colorizer)
add_compile_options(-std=c++14 -Wall)
set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})

# Verify
message(STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  
)
find_package(PCL REQUIRED)
find_package(Eigen3)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system program_options )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_identificator
  CATKIN_DEPENDS cv_bridge pcl_ros roscpp  sensor_msgs std_msgs tf2 tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

)

add_executable(${PROJECT_NAME}_node src/core.cpp src/ros_wrap.cpp src/depth_colorizer_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${EIGEN3_LIBRARIES}
  ${Boost_LIBRARIES}
 )





