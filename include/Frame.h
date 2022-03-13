#include <opencv2/opencv.hpp>
#include <iostream> //debug

#ifndef FRAME
#define FRAME

namespace mrcv {

class Frame {
	/*
	* image
	* focal length
	* K
	* Kinv
	* pose
	* kp, pts
	* des
	*/

	public:
		Frame();
		Frame(const Frame& frame);
		Frame(cv::Mat& image)

		cv::Mat image;
		//cv::Mat K;
}

}
#endif
