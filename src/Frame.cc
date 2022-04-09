
#include "Frame.h"
#include <iostream> // debug
#include <assert.h> // debug, maybe implement MRCVAssert?

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

		// Resize and make both frames grayscale
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
					cv::Point2f pt (point.at<float>(0, 0), point.at<float>(0, 1));
					(frames == 0) ? this->pts1.push_back(pt) : this->pts2.push_back(pt);

					if(run_option == MRCV_DEBUG) {
						allPts++;
						if( point.at<float>(0, 0) > 1.f || point.at<float>(0, 0) < -1.f) wrongPts++;
						if( point.at<float>(0, 1) > 1.f || point.at<float>(0, 1) < -1.f) wrongPts++;
					}
	
				}
			}
		}
		if(run_option == MRCV_DEBUG) std::cout << "Frame " << frames  << "  -> " << allPts << " : " << wrongPts << std::endl;
	}

	// Compute Descriptors

	this->extractor->compute(this->img1, this->kps1, this->des1);
	this->extractor->compute(this->img2, this->kps2, this->des2);

	// Match Keypoints

	std::vector<std::vector<cv::DMatch>> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	matcher->knnMatch(this->des1, this->des2, matches, 2);


	// Filter Matches and save point coordinates
	for( auto i : matches) {
		if( i[0].distance < 0.75*i[1].distance) {
			if(i[0].distance < 32) {
				this->matches.push_back(i[0]);
				this->kptsQuery.push_back( this->pts1[i[0].queryIdx]);
				this->kptsTrain.push_back( this->pts2[i[0].trainIdx]);
			}
		}
	}

	if(run_option == MRCV_DEBUG) std::cout << "Matches  -> " << this->matches.size(); 

	// Compute Essential E matrix and inliners 
	// CAN USE recoverPose() here to skip the singular value decomposition on the essential matrix. It also returns the inliers
	cv::Mat Mask;
	cv::Mat E = cv::findEssentialMat( this->kptsQuery, this->kptsTrain, this->getK(), cv::RANSAC, 0.999, RANSAC_THRESH, Mask);


	// Filter the inliers from the big keypoint lists
	int count = 0;
	for( size_t maskRow = 0; maskRow <= this->kptsQuery.size(); maskRow++) {
		if(!(unsigned int)Mask.at<unsigned char>(maskRow)) {	
			this->kptsQuery.erase( this->kptsQuery.begin() + maskRow );
			this->kptsTrain.erase( this->kptsTrain.begin() + maskRow );
			count++;
		} 
	}
	std::cout << " - " << count << " outliers." << std::endl;


	// Compute Pose from E
	
	//W^-1 = W.t()
	float W_data[] = { 0,-1, 0,
	                   1, 0, 0,
	                   0, 0, 1};

	cv::Mat W(3, 3, CV_64FC1, W_data);
	cv::Mat S, U, Vt, E_hat, err;

	cv::SVDecomp( E, S, U, Vt, cv::SVD::FULL_UV);
	// TODO: Asserts fail
	//assert(cv::determinant(U) > 0);
	//assert(cv::determinant(Vt) < 0);
	
	cv::Mat R = (U * W.t()) * Vt; 
	// TODO: If sum of diagonal R < 0, do normal W
	
	// t = U[:, 2]
	cv::Mat t(1, 3, CV_64FC1);
	for( size_t i = 0; i < U.rows; i++) {
		t.at<double>(0,i) = U.at<double>(i, 2);
	}

	cv::Mat Rt(4, 4, CV_64FC1);
	// fill Rt
	for( size_t i = 0; i <= 2; i++) {
		for( size_t f = 0; f <= 2; f++) {
			Rt.at<double>(i, f) = R.at<double>(i, f);
		}
	}

	Rt.at<double>(0, 3) = t.at<double>(0, 0);
	Rt.at<double>(1, 3) = t.at<double>(0, 1);
	Rt.at<double>(2, 3) = t.at<double>(0, 2);

	for( size_t i = 0; i <= 2; i++) {
		Rt.at<double>(i, 3) = t.at<double>(0, i);
	}

	for( size_t i = 0; i <= 3; i++) {
		(i == 3) ? Rt.at<double>(3, i) = 1 : Rt.at<double>(3, i) = 0;
	}

	std::cout << "\n\n" << Rt << std::endl;
}



cv::Mat Frame::getK() {
	return this->K;
}

} // namespace
