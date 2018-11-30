#include <iostream>
#include <string>  

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
 
int main(int argc, char** argv)
{	
	/*
		Note
		This function is the only method in HighGUI that can fetch and handle events, so it needs to be called periodically for normal event processing 
		unless HighGUI is used within an environment that takes care of event processing.
		Note
		The function only works if there is at least one HighGUI window created and the window is active. 
		If there are several HighGUI windows, any of them can be active. 
	*/
	//the program is runing in 'build' dirctory, however the image is in '../opencv_logo.jpg'!
	Mat image = imread("../opencv_logo.jpg");
	imshow("just used for capture key event!", image);
	
	while(true) {
		int key = waitKey();
		cout << "waitKey()-->key: " << key << endl;
		int key_delay = waitKey(300);
		cout << "waitKey-->key_delay: " << key_delay << endl;
	}
	
	return 0;
}