cmake_minimum_required(VERSION 2.8.3)
project(usb_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

# indigo need OpenCV 2.4
# so I have uninstall previous OpenCV 3.1.0 I used
# in terminator type in "cd /path/to/OpenCV 3.1.0/build/ && sudo make uninstall"
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES usb_camera
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
 DEPENDS system_lib
)

include_directories(
include ${catkin_INCLUDE_DIRS}
)

add_executable(usb_camera src/usb_camera.cpp)
target_link_libraries(usb_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


