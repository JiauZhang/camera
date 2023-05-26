#include <ros/ros.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h> 

using namespace std;
using namespace cv;

int main(int argc, char** argv) 
{ 
	ros::init(argc, argv, "image_publisher"); 
	ros::NodeHandle nh; 
	image_transport::ImageTransport it(nh); 
	image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);  
	cv::Mat image; 
	char key; 
	cv_bridge::CvImage out_msg; 
	ros::Rate loop_rate(30); 

	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		ROS_ERROR("Failed to open camera!");
		return -1;
	}

	double cam_fps = cap.get(CAP_PROP_FPS);
	double cam_width = cap.get(CAP_PROP_FRAME_WIDTH);
	double cam_height = cap.get(CAP_PROP_FRAME_HEIGHT);
	
	ROS_INFO("cam_fps: %.2f", cam_fps);
	ROS_INFO("cam_width: %.2f", cam_width);
	ROS_INFO("cam_height: %.2f", cam_height);

	while (nh.ok() && key != 27) { 
		cap >> image;

		out_msg.header.stamp = ros::Time::now(); 
		out_msg.encoding = cv_bridge::ENCODING_BGR8; 
		out_msg.image = image; 
		pub.publish(out_msg.toImageMsg()); 

		cv::imshow("image", out_msg.image); 

		key = cv::waitKey(1); 
		loop_rate.sleep(); 
	} 

	return 0;
}
