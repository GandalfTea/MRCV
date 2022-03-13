
#include <opencv2/opencv.hpp>
#include "../include/Frame.h"

using namespace std;
using namespace cv;
using namespace mrcv;

int main(int argv, char* argc[]) {
	
	vector<Frame> frames;

	VideoCapture cap("video.mp4");
	if(!cap.isOpened()) {
		cout << "Error opening video" << endl;
		return -1;
	}	

	vector<Frame> images;

	while(1) {
		Mat frame;
		cap >> frame;
		//Ptr<ORB> extractor = ORB::create();

		images.push_back(Frame(frame)); // , extractor));

		if (frame.empty())
			break;

		Mat show = images[-1].data;

		//for(KeyPoint i : images[-1].kps) {
		//	circle(show, Point(i.pt), 1, Scalar(0, 255, 0), FILLED, LINE_8);
		//}

		//imshow("Frame",show);
		char c = (char)waitKey(25);
		if(c==27)
			break;
	}
	cap.release();
	destroyAllWindows();
	return 0;
}
