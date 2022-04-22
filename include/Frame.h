#include <opencv2/opencv.hpp>
#include <iostream> //debug

#ifndef MRCV_FRAME_CORE
#define MRCV_FRAME_CORE

namespace MRCV {

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720

#define FRAME_ROW_STEP FRAME_HEIGHT / 5 
#define FRAME_COL_STEP FRAME_WIDTH / 5

#define MILLIMETERS_TO_PIXELS(F) ((F / SENSOR_WIDTH) * FRAME_WIDTH) * FOCAL_SCALING

#define X0 FRAME_WIDTH / 2
#define Y0 FRAME_HEIGHT / 2
#define SENSOR_HEIGHT 4.29f
#define SENSOR_WIDTH 5.76f
#define FOCAL_SCALING 1

#define Fx MILLIMETERS_TO_PIXELS(27)
#define Fy MILLIMETERS_TO_PIXELS(27)

#define RANSAC_MAXITERS 2000
#define RANSAC_THRESH 3


typedef enum {
	MRCV_DEBUG,
	MRCV_SILENT,
	MRCV_VERBOSE
} RunOption;


typedef enum {
	FRAME_GRAYSCALE_CONVERSION_FAIL,
	FRAME_INCORRECT_IMAGE_MATRIX,
	FRAME_INCORRECT_MATRIX_CHANNELS,
	FRAME_SUBSAMPELING_CONVOLUTION_OUT_OF_RANGE
} FrameError;


typedef enum {
	FRAME_RESIZE_DIFFERENT_ASPECT_RATIO,
	FRAME_EXTRACTION_NO_KEYPOINTS_FOUND,
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


/* This struct holds an individual frame
 * in order to not process the same frame twice. 
 * TODO: Rename*/
struct frame {
	cv::Mat img;
	std::vector<cv::KeyPoint> kps;
	cv::Mat des;
	std::vector<cv::Point2f> pts;
	cv::Mat pose;
	unsigned int imgId;
};

/*
 * The main Frame class holds the triangulated 3D frame computed from 2 video frames
 */
class Frame {
	public:
		Frame() = delete;
		//Frame(const Frame& frame);
		Frame(frame& frame1, frame& frame2, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		Frame(cv::Mat& img1, cv::Mat& img2, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		Frame(cv::Mat& img1, cv::Mat& img2, cv::Mat& img3, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		Frame(cv::Mat& img1, cv::Mat& img2, cv::Mat& img3, cv::Mat& img4, cv::Ptr<cv::ORB>& Extractor, RunOption run_option = MRCV_SILENT);
		//~Frame();
		
		// Internal
		void extract( frame& current );

		unsigned id;
		frame f1, f2;
		cv::Mat show; // image that is shown in debug screen 
		cv::Ptr<cv::ORB>& extractor; // TODO: make general extractor pointer

		std::vector<cv::DMatch> matches;
		std::vector<cv::Point2f> kptsTrain, kptsQuery;
		cv::Mat desTrain;

		// Intrinsic Camera Parameters
		float F;
		cv::Mat K;

		// Extrinsic Camera Parameters
		cv::Mat pose;
		cv::Mat R;
		cv::Mat t;

		// Triangulated 4D Points
		cv::Mat points;

	protected:
		static unsigned mId;
};


}	// namespace
#endif
