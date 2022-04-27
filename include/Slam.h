
#ifndef MRCV_SLAM_CORE
#define MRCV_SLAM_CORE

#include <opencv2/opencv.hpp>
#include <chrono>

#if __has_include(<pangolin/gl/gl.h>)
	#include <pangolin/var/var.h>
	#include <pangolin/var/varextra.h>
	#include <pangolin/gl/gl.h>
	#include <pangolin/gl/gldraw.h>
	#include <pangolin/display/display.h>
	#include <pangolin/display/view.h>
	#include <pangolin/display/default_font.h>
	#include <pangolin/handler/handler.h>
#else
	#define NO_PANGO_LIB
#endif

#if __has_include(<Eigen/Dense>) || __has_include(<eigen3/Eigen/Dense>)
	#include <Eigen/Dense>
	#include <MovementHandler.h>
	//#include <Kalman.h>
#else
	#define NO_EIGEN_LIB
	#define NO_EKF
	#define NO_PANGO_HANDLER
#endif

#if __has_include(<matplotlibcpp.h>) && defined MRCV_GRAPHING
	#include <Python.h>
	#include <matplotlibcpp.h>
#endif

#include <Frame.h>
#include <Map.h>


#namespace MRCV {

struct CameraDetails {
	size_t frame_width;
	size_t frame_height;
	float sensor_width;
	float sensor_height	= 0.f;
	float focal_scaling = 1.f;
	size_t row_step = 5;
	size_t col_step = 5;
	size_t principal_x = frame_width / 2;
	size_t principal_y = frame_height / 2;
};


typedef enum {
	MRCV_SPEEDY,
	MRCV_POWER_SAVING,
	MRCV_DETAILED
} RunMode;


class Slam {

	public:

		Slam( std::string video_path );

		Slam( std::string video_path, CameraDetails cam_details );

		Slam( std::string video_path, size_t frame_width, size_t frame_height, size_t row_step, size_t col_step );

		Slam( std::string video_path, size_t frame_width, size_t frame_height, size_t row_step, 
					size_t col_step float Fx_cm, float Fy_cm, float sensor_width, float sensor_height, 
					float focal_scaling, size_t ransac_cycles, size_t ransac_thresh );

		~Slam();

		// Helpers
		void showVideo();
		void showMap();
		void plotDisplacement();

	private:

		RunMode mode;

		bool show_map = false;
		bool show_video = false;
		bool show_keypoints = false;
		bool show_matches = false;
		
		// Map 3D viewer
		int map_viewer_w = 1000;
		int map_viewer_h = 1000;
		int map_viwerer_depth = 10000;
}




} // namespace
#endif
