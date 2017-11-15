cmake_minimum_required(VERSION 2.8.3)
project(car_data_save)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  tf
  std_msgs
  pcl_conversions
  pcl_ros
)

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS std_msgs pcl_ros
)
link_directories(${PCL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

ADD_EXECUTABLE(car_data_save
  ${PROJECT_SOURCE_DIR}/src/main.cpp
)

TARGET_LINK_LIBRARIES(car_data_save
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)
