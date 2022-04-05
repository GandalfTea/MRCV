
#include <opencv2/opencv.hpp>
#include "../include/Frame.h"
#include <chrono>

using namespace std;
using namespace cv;
using namespace MRCV;

int main(int argv, char* argc[]) {
	
	vector<Frame> frames;

	VideoCapture cap("../data/driving2.mp4");
	if(!cap.isOpened()) {
		cout << "Error opening video" << endl;
		return -1;
	}	

	vector<Frame> images;

	while(1) {
		auto start = std::chrono::system_clock::now();
		Mat frame;
		cap >> frame;
		Ptr<ORB> extractor = ORB::create();

		images.push_back(Frame(frame, extractor));

		if (frame.empty())
			break;

		if( images.size() > 2) {
			Mat show = images.back().data;

			for(KeyPoint i : images.back().kps) {
				circle(show, Point(i.pt), 1, Scalar(0, 255, 0), FILLED, LINE_8);
			}

			auto stop = std::chrono::system_clock::now();
			std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count() << " milliseconds" << std::endl;
			imshow("Frame", show);
			//displayOverlay("Frame", to_string(time.count()), 0);
		}

		char c = (char)waitKey(25);
		if(c==27)
			break;
	}
	cap.release();
	destroyAllWindows();
	return 0;
}
