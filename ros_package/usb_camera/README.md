## usb_camera
This package is used to capture image from my `usb camera`, meanwhile it publish a topic named `/camera/image_raw`. If you want to use this package, you need to do as follows:
 1. Copy this package to your ros workspace.
 2. compile the package
```shell
cd ~/catkin_ws
catkin_make
#start usb_camera node
rosrun usb_camera usb_camera
```
The package will show a window to show the image captured from camera, you can close this property by committing the code `cv::imshow("image", out_msg.image)` in usb_camera.cpp.
