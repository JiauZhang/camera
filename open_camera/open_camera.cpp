#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
 
using namespace cv;
using namespace std;
 
int main()
{
	VideoCapture cap(0);
	
	double cam_fps = cap.get(CV_CAP_PROP_FPS);
	double cam_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double cam_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	cout << "cam_fps: " << cam_fps << endl;
	cout << "cam_width: " << cam_width << endl;
	cout << "cam_height: " << cam_height << endl;
	
	if(!cap.isOpened())
	{
		cout << "open camera failed!" << endl;
		return -1;
	}
	Mat frame;
	Mat edges;
 
	bool stop = false;
	while(!stop)
	{
		cap>>frame;
		imshow("camera", frame);
		if(waitKey(30) >=0)
			stop = true;
	}
	return 0;
}
