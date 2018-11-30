#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if(argc != 2) {
		cout << "Usage: xml_yaml filename" << endl;
		return 1;
	}
	
	string filename = argv[1];
	
	{ //write
		Mat R = Mat_<uchar>::eye(3, 3),
			T = Mat_<double>::zeros(3, 1);
		//MyData m(1);
		FileStorage fs(filename, FileStorage::WRITE);
		
		fs << "One" << 1;
		fs << "strings" << "[";
		fs << "This is a test string" << " ;aslkdjf";
		fs << "]";
		fs << "R" << R;
		fs << "T" << T;	
		
		fs.release();
	}
	
	cout << "wirte complete" << endl;
	
	return 0;
}