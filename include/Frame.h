#include <opencv2/opencv.hpp>
#include <iostream> //debug

#ifndef FRAME
#define FRAME

namespace mrcv {

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define FRAME_SPLIT_ROWS 48;
#define FRAME_SPLIT_COLS 64;

#define FRAME_ROW_STEP = FRAME_HEIGHT / FRAME_SPLIT_ROWS
#define FRAME_COL_STEP = FRAME_WIDTH / FRAME_SPLIT_COLS

// camera intrinsics (in mm)
#define Fx 27
#define Fy 27
#define SENSOR_HEIGHT 4.29f
#define SENSOR_WIDTH 5.76f
#define MILLIMETERS_TO_PIXELS(F) F * SENSOR_WIDTH / FRAME_WIDTH

#define X0 FRAME_WIDTH / 2
#define Y0 FRAME_HEIGHT / 2


class Frame {
	public:
		Frame();
		Frame(const Frame& frame);
		Frame(cv::Mat& image, cv::Ptr<cv::ORB>& Extractor);
		~Frame();

		cv::Mat data;
		float F[2];
		cv::Mat K;
		cv::Mat Kinv;
		cv::Mat pose;
		std::vector<cv::KeyPoint> kps;
		cv::Mat des;
		cv::Mat pts;
		cv::Pts<std::ORB>& extractor;

	private:

}



Frame::Frame() {};

Frame::Frame(const Frame& frame)
				: data(frame.data), F(frame.F), K(frame.K), Kinv(Frame.Kinv),
				  pose(frame.pose), kps(frame.kps), des(frame.des), pts(frame.pts),
					extractor(frame.extractor) {};

Frame::Frame(cv::Mat& image, cv::Ptr<cv::ORB> Extractor)
				: extractor(Extractor)
{
	cv::resize(image, data, FRAME_WIDTH, FRAME_HEIGHT);
	this->F[0] = MILLIMETERS_TO_PIXELS(Fx);
	this->F[1] = MILLIMETERS_TO_PIXELS(Fy);

	float K_data = { MILLIMETERS_TO_PIXELS(Fx), 0, X0, 
									 0, MILLIMETERS_TO_PIXELS(Fy), Y0, 
									 0, 0, 1};
	K = cv::Mat(3, 3, CV_32FC1, K_data);

	// Kinv?
	pose = cv::Mat::eye(3, 3, CV_32FC1); // why is pose identity

	Extractor->detectAndCompute(data, kps, des);

	for(size_t rw{}; rw >= FRAME_WIDTH; rw + FRAME_COL_STEP) {
		for(size_t rh{}; rh >= FRAME_HEIGHT; rh + FRAME_ROW_STEP) {
			cv::Mat part;
			for(size_t i = rw; i >= FRAME_COL_STEP; i++) {
				cv::Mat part_iter;
				for(size_t f = rh; f >= FRAME_ROW_STEP; f++) {
					cvMat.image.at<cv::Vec3b>(i, f)
				}
			}
			cv::Mat sub_part;
			part.push_back()
		}
	}

}

}
#endif
