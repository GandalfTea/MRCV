
#include <opencv2/opencv.hpp>
#include "../include/Frame.h"
#include <chrono>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;
using namespace MRCV;

int main(int argv, char* argc[]) {
	

	VideoCapture cap("../data/driving2.mp4");
	if(!cap.isOpened()) {
		cout << "Error opening video" << endl;
		return -1;
	}	

	pcl::visualization::CloudViewer viewer ("SLAM");
	pcl::PointCloud<plc::PointXYZ> cloud;
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize( cloud.width * cloud.height );

	while(1) {
		auto start = std::chrono::system_clock::now();
		Mat frame;
		cap >> frame;
		if (frame.empty())
			break;

		Ptr<ORB> extractor = ORB::create(200, 1.2f, 8, 5, 0, 2, cv::ORB::HARRIS_SCORE, 40, 20);

		images.push_back(frame);

		if( images.size() > 2) {
			frames.push_back(Frame(images.rbegin()[0], images.rbegin()[1], extractor, MRCV_DEBUG));

			Mat show = frames.back().show;

			auto stop = std::chrono::system_clock::now();
			std::cout << "\t" << std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count() << " ms\n" << std::endl;

			/*
			for(size_t i{}; i <= frames.back().kptsTrain.size(); i++) {
					float query_data [] = { frames.back().kptsQuery[i].x, frames.back().kptsQuery[i].y, 1};
					float train_data [] = { frames.back().kptsTrain[i].x, frames.back().kptsTrain[i].y, 1};
					cv::Mat queryData( 3, 1, CV_32FC1, query_data);
					cv::Mat trainData( 3, 1, CV_32FC1, train_data);

					cv::Mat query = frames.back().K * queryData;
					cv::Mat train = frames.back().K * trainData;

					circle(show, Point(query.at<float>(0, 0), query.at<float>(0, 1)), 1, Scalar(0, 255, 0), 1, LINE_8);
					 TODO: FIX MATCHES DISPLAY
					line  (show, Point(query.at<float>(0, 0), query.at<float>(0, 1)), 
									  	 Point(train.at<float>(0, 0), train.at<float>(0, 1)), 
										   Scalar(255, 255, 255), 0.01);
											 
			}
			for( auto i : frames.back().matches) {
					line (show, frames.back().kps1[i.queryIdx].pt, 
											frames.back().kps2[i.trainIdx].pt,
										  Scalar(255, 255, 255), 0.01);
			}

			*/

			for( size_t i{} ; i <= frames.back().points.rows; i++) {
				cloud.points[i].x = frames.back().points.at<double>(i, 0);
				cloud.points[i].y = frames.back().points.at<double>(i, 1);
				cloud.points[i].z = frames.back().points.at<double>(i, 2);
			}

			}

			//imshow("Frame", show);
			viewer.showCloud(cloud);
		}

		char c = (char)waitKey(25);
		if(c==27)
			break;
	}
	cap.release();
	destroyAllWindows();
	return 0;
}
