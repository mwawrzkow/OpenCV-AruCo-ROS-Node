cmake_minimum_required(VERSION 3.0.2)
project(camimg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES missioncontroll
#  CATKIN_DEPENDS geometry_msg roscpp
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui aruco)
add_executable(imageRaw src/main.cpp src/EtQ.cpp)
target_link_libraries(imageRaw ${catkin_LIBRARIES} ${OpenCV_LIBS})
