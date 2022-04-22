
#include <opencv2/opencv.hpp>
#include <chrono>

#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>

#include <Eigen/Dense>
#include <Python.h>
#include <matplotlibcpp.h>

#include "../include/Frame.h"
#include "../include/MovementHandler.h"
#include "../include/Map.h"

using namespace std;
using namespace cv;
using namespace MRCV;


int main(int argv, char* argc[]) {
	
	VideoCapture cap("../data/driving2.mp4");
	if(!cap.isOpened()) {
		cout << "Error opening video" << endl;
		return -1;
	}	

	pangolin::CreateWindowAndBind("SLAM", 1000, 1000);
	glEnable(GL_DEPTH_TEST);
	pangolin::OpenGlRenderState s_cam (
		pangolin::ProjectionMatrix( 1000, 1000, 600, 600, 320, 249, 0.1, 10000),
		pangolin::ModelViewLookAt(0,0, -5.0, 0,0,0, 0,1,0)		
	);

	pangolin::MovementHandler handler(s_cam);

	pangolin::View& d_cam = pangolin::CreateDisplay()
					.SetBounds(0.0, 1.0, 0.0, 1.0, -1000.0f/1000.0f)
					.SetHandler( &handler );

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	vector<cv::Mat> images;
	vector<frame> f;
	vector<Frame> frames;
	vector<Mat> poses;
	vector<cv::Point3f> points;

	vector<double> x_displacement;
	vector<double> y_displacement;
	vector<double> z_displacement;

	float camera_rotation = 0.f;

	Map Map;

	while( !pangolin::ShouldQuit() ) {
		auto start = std::chrono::system_clock::now();
		Mat f;
		cap >> f;
		if (f.empty())
			break;

		images.push_back(f);

		Ptr<ORB> extractor = ORB::create(200, 1.2f, 8, 5, 0, 2, cv::ORB::HARRIS_SCORE, 40, 20);

		if( images.size() >= 2) {
			frame f1;
			frame f2;
			if(images.size() == 2) {
				f1.img = images.rbegin()[1];
			} else {
				f1 = frames.back().f2;
			}
			f2.img = images.rbegin()[0];

			frames.push_back(Frame(f1, f2, extractor, MRCV_SILENT));
			//poses.push_back(frames.back().pose);

			//Mat show = frames.back().show;

			auto stop = std::chrono::system_clock::now();
			std::cout << "\t" << std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count() << " ms\n" << std::endl;

			Map.update( frames.back().points, frames.back().desTrain );
			std::cout << "MAP SIZE : " << Map.size() << std::endl;

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

			// Render Points
			
			d_cam.Activate(s_cam);
			glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		

			for( size_t i = 0; i <= frames.back().points.cols; i++) {
				float x = frames.back().points.at<float>(i, 0) * 10; 
				float y = frames.back().points.at<float>(i, 1) * 10; 
				float z = frames.back().points.at<float>(i, 2) * 10; 
				points.push_back(cv::Point3f(x, y, z));
			}

			for( auto i : points) {
				glEnable(GL_POINT_SMOOTH);
				glPointSize(1);
				glColor3d(1.0, 1.0, 1.0);
				glBegin(GL_POINTS);
				glVertex3d(i.x, i.y, i.z); 
				glEnd();
			}

			

			// Show camera pose 

			for( auto i : poses ) {
				glEnable(GL_POINT_SMOOTH);
				glPointSize(5);
				glColor3d(0, 1.0, 0);
				glBegin(GL_POINTS);
				float x = i.at<double>(3, 0) * 10 ; 
				float y = i.at<double>(3, 1) * 10 ; 
				float z = i.at<double>(3, 2) * 10 ; 
				glVertex3d(x, y, z); 
				glEnd();
			
			}


			// Calculate displacement
			// TODO: Move this to the Kalman filter
			// TODO: Derive Velocity and Acceleration
			
			if( frames.size() > 2) {
				cv::Mat vec1, vec2;
				cv::Mat R_disp;

				cv::Mat R1 = frames.rbegin()[0].R;
				cv::Mat R2 = frames.rbegin()[1].R;
				cv::Mat t1 = frames.rbegin()[0].t;
				cv::Mat t2 = frames.rbegin()[1].t;

				cv::Rodrigues( R1, vec1 );
				cv::Rodrigues( R2, vec2 );

				cv::Mat R1toR2 = R2 * R1.t();
				cv::Rodrigues( R1toR2, R_disp );
				cv::Mat t_disp = -R1toR2 * t1 + t2; 

				/*
				std::cout.precision(50);
				std::cout << std::endl;
				for( size_t i = 0; i <= R_disp.cols+1; i ++) {
					std::cout << std::fixed << R_disp.at<double>(0, i) << std::endl;
				}

				for( size_t i = 0; i <= t_disp.cols+1; i ++) {
					std::cout << std::fixed << t_disp.at<double>(0, i) << std::endl;
				}
				*/
			}


			// Apply Frame Pose to Camera

			cv::Mat Rt = frames.rbegin()[0].pose;

			Eigen::Matrix<double, 4, 4> NewRt {
				{Rt.at<double>(0,0), Rt.at<double>(0,1), Rt.at<double>(0,2), Rt.at<double>(0,3)},
				{Rt.at<double>(1,0), Rt.at<double>(1,1), Rt.at<double>(1,2), Rt.at<double>(1,3)},
				{Rt.at<double>(2,0), Rt.at<double>(2,1), Rt.at<double>(2,2), Rt.at<double>(2,3)},
				{0, 0, 0, 1},
			};

			auto RT = pangolin::OpenGlMatrix(NewRt);
			//auto iRt = RT.Inverse();
			//const pangolin::OpenGlMatrix T_vc = s_cam.GetModelViewMatrix() * iRt;
			//s_cam.SetModelViewMatrix(T_vc);
			//pangolin::OpenGlMatrix& cam = s_cam.GetModelViewMatrix();
			//std::cout << cam << std::endl;
			//s_cam.Follow(RT, true);
			
			//imshow("Frame", show);

		}
		pangolin::FinishFrame();

	}
	namespace plt = matplotlibcpp;
	plt::plot(x_displacement);
	plt::plot(y_displacement);
	plt::plot(z_displacement);
	plt::show();
	destroyAllWindows();
	return 0;

}
