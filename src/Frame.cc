
#include "Frame.h"

#include <iostream> // debug
namespace MRCV
{

/*
Frame::Frame(const Frame& frame)
				: data(frame.data), F(frame.F), K(frame.getK()), Kinv(frame.getKinv()),
				  pose(frame.pose), kps(frame.kps), des(frame.getDes()), pts(frame.getPts()),
					extractor(frame.extractor) {};
*/


Frame::Frame(cv::Mat& image, cv::Ptr<cv::ORB>& Extractor)
				: extractor(Extractor)
{
	cv::Mat resized;
	cv::resize(image, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
	cv::cvtColor(resized, this->data, cv::COLOR_RGB2GRAY);

	this->F = MILLIMETERS_TO_PIXELS(Fx);

	float K_data [9] = { MILLIMETERS_TO_PIXELS(Fx), 0, X0, 
									  0, MILLIMETERS_TO_PIXELS(Fy), Y0, 
									  0, 0, 1};

	K = cv::Mat(3, 3, CV_32FC1, K_data);
	Kinv = K.inv();


	pose = cv::Mat::eye(3, 3, CV_32FC1); // why is pose identity

	// Split image for better keypoint extraction
	for(size_t rw = 0; rw < FRAME_WIDTH; rw += FRAME_COL_STEP) {
		for(size_t rh = 0; rh < FRAME_HEIGHT; rh += FRAME_ROW_STEP) {

			// Create subimage matrix
			cv::Mat subimage ( FRAME_ROW_STEP, FRAME_COL_STEP, CV_8UC1 );
			for(size_t i = 0; i < FRAME_ROW_STEP; i++) {
				for(size_t f = 0; f < FRAME_COL_STEP; f++) {
					subimage.at<unsigned char>(i,f) = this->data.at<unsigned char>(rh + i, rw + f);
				}
			}
			std::vector<cv::KeyPoint> ukps{};
			//std::vector<?> upts;
			this->extractor->detect(subimage, ukps);


			for( auto i : ukps ) {

				// Recalibrate the keypoint coordinates for the big image
				i.pt.x += rw;
				i.pt.y += rh;
				this->kps.push_back(i);

				// Normalize the point coordinates
				// by doing the dot product with the inverse K
				float val [3] = { i.pt.x, i.pt.y, 1 };
				cv::Mat normal( 3, 1, CV_32FC1, val);
				cv::Mat point = this->Kinv * normal;
				//std::cout << point.t() << std::endl;
				this->pts.push_back(point.t());
			}
		}
	}

	this->extractor->compute(this->data, this->kps, this->des);


}

}
