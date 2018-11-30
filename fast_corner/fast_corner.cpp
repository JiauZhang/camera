#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
 
using namespace cv;
using namespace std;
 
int main()
{
	VideoCapture cap(0);
	if(!cap.isOpened()) {
		cout << "open camera failed!" << endl;
		return -1;
	}
	
	Mat frame; 
	bool stop = false;
	while(!stop) {
		cap>>frame;
		
		vector<KeyPoint> keypoints;
		 Ptr<FastFeatureDetector> detector = cv::FastFeatureDetector::create();
		detector->detect(frame, keypoints);
		drawKeypoints(frame, keypoints, frame);		
		imshow("camera", frame);
		//Parameters:	delay – Delay in milliseconds. 0 is the special value that means “forever”.
		if(waitKey(50) >=0)
			stop = true;
	}
	
	return 0;
}