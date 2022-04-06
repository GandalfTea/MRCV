#include <opencv2/opencv.hpp>
#include <iostream> //debug

#ifndef MRCV_FRAME_CORE
#define MRCV_FRAME_CORE

namespace MRCV {

#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720
#define FRAME_ROW_STEP FRAME_HEIGHT / 5 
#define FRAME_COL_STEP FRAME_WIDTH / 5

#define X0 FRAME_WIDTH / 2
#define Y0 FRAME_HEIGHT / 2
#define Fx 27
#define Fy 27
#define SENSOR_HEIGHT 4.29f
#define SENSOR_WIDTH 5.76f
#define FOCAL_SCALING 1

#define MILLIMETERS_TO_PIXELS(F) ((F / SENSOR_WIDTH) * FRAME_WIDTH) * FOCAL_SCALING

typedef enum {
	MRCV_DEBUG,
	MRCV_SILENT
} RunOption;


typedef enum {
	FRAME_INCORRECT_IMAGE_MATRIX,
	FRAME_RESIZE_INCORRECT_ASPECT_RATIO,
	FRAME_INCORRECT_MATRIX_CHANNELS,
	FRAME_GRAYSCALE_CONVERSION_FAIL,
	FRAME_SUBSAMPELING_CONVOLUTION_OUT_OF_RANGE
} FrameError;


typedef enum {
	FRAME_INCORRECT_NORMALIZATION
} Allert;



class FrameException {
	public:
		FrameError error_;
		explicit FrameException( FrameError error) : error_(error) {}
};

class FrameAlert {
	public:
		FrameError error_;
		explicit FrameAlert( FrameError error) : error_(error) {}
};


class Frame {
	public:
		Frame() = delete;
		//Frame(const Frame& frame);
		Frame(cv::Mat& img1, cv::Mat& img2, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		Frame(cv::Mat& img1, cv::Mat& img2, cv::Mat& img3, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		Frame(cv::Mat& img1, cv::Mat& img2, cv::Mat& img3, cv::Mat& img4, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		//~Frame();

		cv::Mat img1;
		cv::Mat img2;
		float F;
		//cv::Mat pose;
		cv::Ptr<cv::ORB>& extractor;
		std::vector<cv::KeyPoint> kps1;
		std::vector<cv::KeyPoint> kps2;
		cv::Mat des1;
		cv::Mat des2;
		std::vector<cv::Mat> pts1;
		std::vector<cv::Mat> pts2;

		std::vector<cv::DMatch> matches;
		std::vector<cv::Point2f> kptsTrain, kptsQuery;

		// Helpers
		cv::Mat getK();
		cv::Mat getKinv();

	private:
		cv::Mat K;
		cv::Mat Kinv;

		static unsigned mId;
};


}	// namespace
#endif
