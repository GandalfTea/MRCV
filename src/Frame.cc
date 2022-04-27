
#include "Frame.h"
#include <iostream> // debug
#include <assert.h> // debug, maybe implement MRCVAssert?

namespace MRCV
{

unsigned Frame::mId = 0;

/*
Frame::Frame(const Frame& frame)
				: data(frame.data), F(frame.F), K(frame.getK()), Kinv(frame.getKinv()),
				  pose(frame.pose), kps(frame.kps), des(frame.getDes()), pts(frame.getPts()),
					extractor(frame.extractor) {};
*/




Frame::Frame(frame& frame1, frame& frame2, cv::Ptr<cv::ORB>& Extractor, RunOption run_option)
				: extractor(Extractor), id(mId++), f1(frame1), f2(frame2)
{

/*	Set time  */

		this->time; 



/*  Resize image into FRAME_WIDTH X FRAME_HEIGHT
		Change color into GRAYSCALE.
		Change color of 'show' image back into RGB        */

	try {
		cv::Mat resized;
		if( id == 0 ) cv::resize(this->f1.img, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
		if( id == 0 ) cv::cvtColor(resized, this->f1.img, cv::COLOR_RGB2GRAY);
		cv::resize(this->f2.img, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
		cv::cvtColor(resized, this->f2.img, cv::COLOR_RGB2GRAY);
		cv::cvtColor(this->f1.img, this->show, cv::COLOR_GRAY2RGB);
	
	} catch( cv::Exception& e) {
		std::cout << "OpenCV Exception : " << (const char*)e.what() << std::endl;
		throw FrameException( FRAME_GRAYSCALE_CONVERSION_FAIL);
	}



/*  Create CAMERA INTRINSICS MATRIX ( K )
		Also create the inverse         ( Kinv )
    They are used to normalize and denormalize the coords.    */

	this->F = Fx;
	float K_data [9] = { Fx, 0, X0, 
	                     0, Fy, Y0, 
                       0,  0, 1  };
	K = cv::Mat(3, 3, CV_32FC1, K_data);



/*  KEYPOINT EXTRACTION and DESCRIPTOR COMPUTING
 
		Using Orb for both extraction and computing descriptors.
		We split the image into FRAME_COL_STEP x FRAME_ROW_STEP parts for better extraction.
		This should give us ~2500 - 3000 keypoints per frame.
		KNNMatch with BFMatcher to match.                  */

	#ifdef MRCV_DEBUG
	int allPts = 0, wrongPts = 0;
	#endif

	if( id == 0 ) {
		// First pair of frames, need to process both of them
		extract(f1);
		extract(f2);
		this->extractor->compute(f1.img, f1.kps, f1.des);
		this->extractor->compute(f2.img, f2.kps, f2.des);

	} else {
		// Only compute the second frame, first one was computed as second last cycle
		extract(f2);
		this->extractor->compute(f2.img, f2.kps, f2.des);
	}


	// KEYPOINT MATCHING 
	cv::Mat desTrainTemp; // temporarely hold the descriptors before pushing them into the final mat when filtering inliers
	std::vector<std::vector<cv::DMatch>> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	matcher->knnMatch(f1.des, f2.des, matches, 2);


	// Filter Matches and save point coordinates
	for( auto i : matches) {
		if( i[0].distance < 0.75*i[1].distance) {
			if(i[0].distance < 32) {
				this->matches.push_back(i[0]);
				this->kptsQuery.push_back( this->f1.pts[i[0].queryIdx]);
				this->kptsTrain.push_back( this->f2.pts[i[0].trainIdx]);

				// Ordered descriptors for Map check
				cv::Mat row2 = f2.des.row(i[0].trainIdx);
				desTrainTemp.push_back( row2 );
			}
		}
	}

	if(run_option == MRCV_DEBUG) std::cout << "Matches  -> " << this->matches.size(); 



	// Compute Essential E matrix and inliners 
	cv::Mat Mask;
	cv::Mat E = cv::findEssentialMat( this->kptsQuery, this->kptsTrain, this->K, cv::RANSAC, 0.999, RANSAC_THRESH, Mask);


	// Filter the inliers from the big keypoint lists
	int count = 0;
	for( size_t maskRow = 0; maskRow <= kptsQuery.size(); maskRow++) {
		if(!(unsigned int)Mask.at<unsigned char>(maskRow)) {	
			this->kptsQuery.erase( this->kptsQuery.begin() + maskRow );
			this->kptsTrain.erase( this->kptsTrain.begin() + maskRow );
			count++;

		} else {
			if(maskRow < desTrainTemp.rows) {
				cv::Mat row = desTrainTemp.row(maskRow);
				this->desTrain.push_back( row );
			}
		}
	}



	// Compute Rt using OpenCV helpers 
	cv::Mat opencv_R, opencv_t, opencv_inliers;
	cv::recoverPose( E, this->kptsQuery, this->kptsTrain, this->K, opencv_R, opencv_t, opencv_inliers);
	this->R = opencv_R;
	this->t = opencv_t;




	// Manualy Compute Pose from E
	
  /*

	//W^-1 = W.t()
	float W_data[] = { 0,-1, 0,
		        				 1, 0, 0,
				        		 0, 0, 1};

	cv::Mat W(3, 3, CV_64FC1, W_data);
	cv::Mat S, U, Vt, E_hat, err;

	cv::SVDecomp( E, S, U, Vt, cv::SVD::FULL_UV);

	if(cv::determinant(U) > 0)   U.col(2) *= -1;
	if(cv::determinant(Vt) < 0) Vt.row(2) *= -1;

	cv::Mat R = U * W * Vt; 
	
	// R sum of diagonal > 0
	float sum = 0.0f;
	for(size_t i = 0; i < R.cols; i++) {
		sum += R.at<float>(i,i);
	}

	cv::Mat rvec;
	cv::Rodrigues(R, rvec);

	if (sum < 0) {
		R = U * W.t() * Vt;
	}


	cv::Mat t(1, 3, CV_64FC1);
//	for( size_t i = 0; i < U.rows; i++) {
//		t.at<double>(0,i) = U.at<double>(i, 2);
//	}

	cv::Mat tskew = U*W*U.t();
	double tvalues [] = { tskew.at<double>(2,1), tskew.at<double>(0,2), tskew.at<double>(1,0)};
	cv::Mat ts2(1, 3, CV_64FC1, tvalues);
	t.at<double>(0,0) = tskew.at<double>(2,1);
	t.at<double>(0,1) = tskew.at<double>(0,2);
	t.at<double>(0,2) = tskew.at<double>(1,0);
	*/




/* Extrinsic Camera Parameters
	 This brings the world coordinates into camera coordinates:
	 Pcamera = Pose * Pworld

	 The translation matrix represents the position of the world origin in relation to the camera-centered coordinates.
	 To get the camera position relative to the origin, we must calculate C:
	 C = -R.t() * t

	 TODO: Compute C matrix and maybe H 					*/

	cv::Mat Rt(3, 4, CV_64FC1);
	for( size_t i = 0; i <= 2; i++) {
		for( size_t f = 0; f <= 2; f++) {
			Rt.at<double>(i, f) = opencv_R.at<double>(i, f);
		}
	}

	for( size_t i = 0; i <= 2; i++) {
		Rt.at<double>(i, 3) = opencv_t.at<double>(0, i);
	}

	/*
	for( size_t i = 0; i <= 3; i++) {
		(i == 3) ? Rt.at<double>(3, i) = 1 : Rt.at<double>(3, i) = 0;
	}
	 */
	this->pose = Rt;

	// Triangulation 
	cv::Mat Knew;
	this->K.convertTo(Knew, CV_64FC1);
	cv::Mat wat = Knew * Rt;
	
	cv::Mat pts4d(1, this->kptsQuery.size(), CV_64FC4);

	cv::triangulatePoints( wat, wat, this->kptsQuery, this->kptsTrain, pts4d);


	/*

	for( size_t i = 0; i < pts4d.cols; i++) {

		auto x = pts4d.at<double>(i, 0);
		auto y = pts4d.at<double>(i, 1);
		auto z = pts4d.at<double>(i, 2);
		auto w = pts4d.at<double>(i, 3);

		if( w != 0 ) { 
			std::cout << x << std::endl;
			std::cout << y << std::endl;
			std::cout << z << std::endl;
			std::cout << w << std::endl;
			double r1 = x / w;
			double r2 = y / w;
			double r3 = z / w;
			double r4 = w / w;
			std::cout << r1 << std::endl;
			std::cout << r2 << std::endl;
			std::cout << r3 << std::endl;
			std::cout << r4 << std::endl;
			std::cout << std::endl << std::endl;
			pts4d.at<double>(i, 0) = r1;
			pts4d.at<double>(i, 1) = r2;
			pts4d.at<double>(i, 2) = r3;
			pts4d.at<double>(i, 3) = r4;
		}
	}
	*/

	pts4d = 100 * pts4d; // scaling

	// filter points behind the camera
	cv::Mat good_pts4d = pts4d.col(0);
	for( size_t i = 0; i < pts4d.cols; i++) {
		if( pts4d.at<double>(i, 1) > 0) {
			cv::Mat pt;
			pts4d.col(i).copyTo(pt);
			good_pts4d.push_back(pt);
		}
	}
	good_pts4d = good_pts4d.reshape(good_pts4d.cols, 4);
	this->points = good_pts4d;
}




void Frame::extract( frame& current ) {

	for(size_t rw = 0; rw < FRAME_WIDTH; rw += FRAME_COL_STEP) {
		for(size_t rh = 0; rh < FRAME_HEIGHT; rh += FRAME_ROW_STEP) {

			// Draw subimage borders
			cv::line (show, cv::Point(0, rh), 
											cv::Point(FRAME_WIDTH, rh), 
								 			cv::Scalar(0, 0, 0), 0);
			cv::line (show, cv::Point(rw, 0), 
											cv::Point(rw, FRAME_HEIGHT), 
								 			cv::Scalar(0, 0, 0), 0);

			// Create Subimage matrix
			cv::Mat subimage ( FRAME_ROW_STEP, FRAME_COL_STEP, CV_8UC1 );
			for(size_t i = 0; i < FRAME_ROW_STEP; i++) {
				for(size_t f = 0; f < FRAME_COL_STEP; f++) {
					try {
						subimage.at<unsigned char>(i,f) = current.img.at<unsigned char>(rh + i, rw + f);

					} catch(cv::Exception& e) {
						std::cout << "OpenCV Exception : " << (const char*)e.what() << std::endl;
						// TODO: check for index out of range, channel and size errors
						throw FrameException(FRAME_SUBSAMPELING_CONVOLUTION_OUT_OF_RANGE);	
					}
				}
			}

			// Extract Keypoints
			std::vector<cv::KeyPoint> ukps{};
			this->extractor->detect(subimage, ukps);
	
			for( auto i : ukps ) {

				// Recalibrate the keypoint coordinates for the big image
				i.pt.x += rw;
				i.pt.y += rh;
				current.kps.push_back(i);
	
				// Normalize the point coordinates
				float val [3] = { i.pt.x, i.pt.y, 1 };
				cv::Mat normal( 3, 1, CV_32FC1, val);
				cv::Mat point = this->K.inv() * normal;
				cv::Point2f pt (point.at<float>(0, 0), point.at<float>(0, 1));
				current.pts.push_back(pt);

#ifdef MRCV_DEBUG
					allPts++;
					if( point.at<float>(0, 0) > 1.f || point.at<float>(0, 0) < -1.f) wrongPts++;
					if( point.at<float>(0, 1) > 1.f || point.at<float>(0, 1) < -1.f) wrongPts++;
#endif
				}
			}
		}
	}


} // namespace
