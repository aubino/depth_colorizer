cmake_minimum_required(VERSION 3.0.2)
project(depth_colorizer)
add_compile_options(-std=c++14 -Wall)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3)

add_definitions(${PCL_DEFINITIONS})


generate_messages(
   DEPENDENCIES
   sensor_msgs   
   std_msgs
   geometry_msgs
 )



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_colorizer
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf tf2 tf2_ros
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node )

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

#############
## Install ##
#############


