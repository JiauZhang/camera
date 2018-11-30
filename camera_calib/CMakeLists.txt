cmake_minimum_required( VERSION 2.8 )
project( camera_calib )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

# add_executable( camera_calib camera_calib.cpp )
# 链接OpenCV库
# target_link_libraries( camera_calib ${OpenCV_LIBS} )


add_executable( calibration calibration.cpp )
#链接OpenCV库
target_link_libraries( calibration ${OpenCV_LIBS} )

# add_executable( camera_calibration camera_calibration.cpp )
# # 链接OpenCV库
# target_link_libraries( camera_calibration ${OpenCV_LIBS} )