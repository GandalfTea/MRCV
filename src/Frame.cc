
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


Frame::Frame(cv::Mat& img1, cv::Mat& img2, cv::Ptr<cv::ORB>& Extractor, RunOption run_option)
				: extractor(Extractor)
{
	cv::Mat resized;
	try {

		// Check for changing of aspect ration. This is mainly for beginners and should be overwritable
		//if (FRAME_WIDTH / FRAME_HEIGHT != image.rows / image.cols) throw FrameException( FRAME_RESIZE_INCORRECT_ASPECT_RATIO);

		cv::resize(img1, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
		cv::cvtColor(resized, this->img1, cv::COLOR_RGB2GRAY);

		cv::resize(img2, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
		cv::cvtColor(resized, this->img2, cv::COLOR_RGB2GRAY);

	} catch( cv::Exception& e) {

		const char* msg = e.what();
		std::cout << "OpenCV Exception : " << msg << std::endl;
		throw FrameException( FRAME_GRAYSCALE_CONVERSION_FAIL);
	}

	this->F = MILLIMETERS_TO_PIXELS(Fx);

	float K_data [9] = { MILLIMETERS_TO_PIXELS(Fx), 0, X0, 
									  0, MILLIMETERS_TO_PIXELS(Fy), Y0, 
									  0, 0, 1};

	K = cv::Mat(3, 3, CV_32FC1, K_data);
	Kinv = K.inv();

	//pose = cv::Mat::eye(3, 3, CV_32FC1); // why is pose identity

	for(int frames = 0; frames <= 1; frames++){

		std::cout << "FRAME INDEX : " << frames << std::endl;

		cv::Mat image;
		(frames == 0) ? image = this->img1 : image = this->img2;

		int allPts = 0, wrongPts = 0;

		// Split image for better keypoint extraction
		for(size_t rw = 0; rw < FRAME_WIDTH; rw += FRAME_COL_STEP) {
			for(size_t rh = 0; rh < FRAME_HEIGHT; rh += FRAME_ROW_STEP) {

				// Create Subimage matrix
				cv::Mat subimage ( FRAME_ROW_STEP, FRAME_COL_STEP, CV_8UC1 );
				for(size_t i = 0; i < FRAME_ROW_STEP; i++) {
					for(size_t f = 0; f < FRAME_COL_STEP; f++) {
						try {
							subimage.at<unsigned char>(i,f) = image.at<unsigned char>(rh + i, rw + f);
	
						} catch(cv::Exception& e) {
	
							const char* msg = e.what();
							std::cout << "OpenCV Exception : " << msg << std::endl;
	
							// check for index out of range, channel and size errors
							throw FrameException(FRAME_SUBSAMPELING_CONVOLUTION_OUT_OF_RANGE);	
						}
					}
				}

				std::vector<cv::KeyPoint> ukps{};
				this->extractor->detect(subimage, ukps);
	
				for( auto i : ukps ) {
	
					// Recalibrate the keypoint coordinates for the big image
					i.pt.x += rw;
					i.pt.y += rh;
					(frames == 0) ? this->kps1.push_back(i) : this->kps2.push_back(i);
	
					// Normalize the point coordinates
					float val [3] = { i.pt.x, i.pt.y, 1 };
					cv::Mat normal( 3, 1, CV_32FC1, val);
					cv::Mat point = this->Kinv * normal;
					(frames == 0) ? this->pts1.push_back(point.t()) : this->pts2.push_back(point.t());

					if(run_option == MRCV_DEBUG) {
						allPts++;
						if( point.at<float>(0, 0) > 1.f || point.at<float>(0, 0) < -1.f) wrongPts++;
						if( point.at<float>(0, 1) > 1.f || point.at<float>(0, 1) < -1.f) wrongPts++;
					}
	
				}
			}
		}
		if(run_option == MRCV_DEBUG) std::cout << "Keypoints : " << wrongPts << " : " << allPts << std::endl;
	}

	// Compute Descriptors

	this->extractor->compute(this->img1, this->kps1, this->des1);
	this->extractor->compute(this->img2, this->kps2, this->des2);

	std::vector<std::vector<cv::DMatch>> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	matcher->knnMatch(this->des1, this->des2, matches, 2);

	for( auto i : matches) {
		if( i[0].distance < 0.75*i[1].distance) {
			if(i[0].distance < 32) {
				this->matches.push_back(i[0]);
				this->kptsQuery.push_back( this->kps1[i[0].queryIdx].pt);
				this->kptsTrain.push_back( this->kps2[i[0].trainIdx].pt);
			}
		}
	}

	if(run_option == MRCV_DEBUG) std::cout << "Matches : " << this->matches.size() << std::endl; 


}

} // namespace
