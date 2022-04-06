
#include <opencv2/opencv.hpp>
#include "../include/Frame.h"
#include <chrono>

using namespace std;
using namespace cv;
using namespace MRCV;

int main(int argv, char* argc[]) {
	

	VideoCapture cap("../data/driving2.mp4");
	if(!cap.isOpened()) {
		cout << "Error opening video" << endl;
		return -1;
	}	

	vector<cv::Mat> images;
	vector<Frame> frames;

	while(1) {
		auto start = std::chrono::system_clock::now();
		Mat frame;
		cap >> frame;
		if (frame.empty())
			break;

		Ptr<ORB> extractor = ORB::create();

		images.push_back(frame);

		if( images.size() > 2) {
			frames.push_back(Frame(images.rbegin()[0], images.rbegin()[1], extractor, MRCV_DEBUG));

			Mat show = frames.back().img1;

			for(KeyPoint i : frames.back().kps1) {
				circle(show, Point(i.pt), 2, Scalar(0, 255, 0), 1, LINE_8);
			}

			for(size_t i{}; i <= frames.back().kptsTrain.size(); i++) {
					line(show, frames.back().kptsQuery[i], frames.back().kptsTrain[i], Scalar(0, 0, 255), 0.1, LINE_AA);
			}

			auto stop = std::chrono::system_clock::now();
			std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count() << " milliseconds\n" << std::endl;

			imshow("Frame", frames.back().img1);
		}

		char c = (char)waitKey(25);
		if(c==27)
			break;
	}
	cap.release();
	destroyAllWindows();
	return 0;
}
