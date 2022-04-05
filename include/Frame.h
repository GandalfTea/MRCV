#include <opencv2/opencv.hpp>
#include <iostream> //debug

#ifndef MRCV_FRAME_CORE
#define MRCV_FRAME_CORE

namespace MRCV {

#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720
#define FRAME_ROW_STEP FRAME_HEIGHT / 5 
#define FRAME_COL_STEP FRAME_WIDTH / 5

				// 32 x 48

#define X0 FRAME_WIDTH / 2
#define Y0 FRAME_HEIGHT / 2
#define Fx 27
#define Fy 27
#define SENSOR_HEIGHT 4.29f
#define SENSOR_WIDTH 5.76f

#define MILLIMETERS_TO_PIXELS(F) F * SENSOR_WIDTH / FRAME_WIDTH



class Frame {
	public:
		Frame() = delete;
		//Frame(const Frame& frame);
		Frame(cv::Mat& image, cv::Ptr<cv::ORB>& Extractor);
		//~Frame();

		cv::Mat data;
		float F;
		cv::Mat pose;
		cv::Ptr<cv::ORB>& extractor;
		std::vector<cv::KeyPoint> kps;

		// Helpers
		cv::Mat getK();
		cv::Mat getKinv();
		cv::Mat getDes();
		cv::Mat getPts();

	private:
		cv::Mat K;
		cv::Mat Kinv;
		cv::Mat des;
		cv::Mat pts;
};


}	// namespace
#endif
