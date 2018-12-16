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

	VideoCapture cap(0);

	if(!cap.isOpened())
	{
		cout << "open camera failed!" << endl;
		return -1;
	}
	
	double cam_fps = cap.get(CV_CAP_PROP_FPS);
	double cam_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double cam_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	cout << "cam_fps: " << cam_fps << endl;
	cout << "cam_width: " << cam_width << endl;
	cout << "cam_height: " << cam_height << endl;	



	ros::init(argc, argv, "image_publisher"); 
	ros::NodeHandle nh; 
	image_transport::ImageTransport it(nh); 
	image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);  
	cv::Mat image; 
	char key; 
	cv_bridge::CvImage out_msg; 
	ros::Rate loop_rate(30); 

	while (nh.ok() && key != 27) { 

			cap>>image;

			out_msg.header.stamp = ros::Time::now(); 
			out_msg.encoding = sensor_msgs::image_encodings::BGR8; 
			out_msg.image = image; 
			pub.publish(out_msg.toImageMsg()); 

			cv::imshow("image", out_msg.image); 

			key = cv::waitKey(1); 
			loop_rate.sleep(); 
	} 
	return 0;
}








