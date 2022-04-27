
#ifndef MRCV_MAP_H
#define MRCV_MAP_H

#include <Frame.h>
//#include <Kalman.h>

#define MRCV_DETAILED // hardcode for now

namespace MRCV {

typedef unsigned char uchar;

#if defined MRCV_POWER_SAVING || defined MRCV_SPEEDY
	#define MAP_MAX_SIZE 100000 * sizeof(Point)*4
#elif defined MRCV_DETAILED
	#define MAP_MAX_SIZE 1000000 * sizeof(double)*4
#endif


struct Point {
	cv::Point3f pt;
	std::vector<cv::Point3f> observations;
	cv::Mat des;
	float c;
};

class Map {
	public:
		Map();
		~Map();

		void update( cv::Mat pts, cv::Mat des );
		void update( std::vector<cv::Point3f> pts, std::vector<uchar> des );
		void update( const Frame& frame );
		void clear();
		void draw();
		void video();

		// Helpers
		std::vector<std::vector<cv::DMatch>> contains( cv::Mat des ); 
		void add( Point a );
		int size();

		int mPointsFound = 0;
		int mNewPoints = 0;
		int mRepeatingPoints = 0;
		cv::Ptr<cv::DescriptorMatcher> matcher; 
		cv::Mat des;
		std::vector<Point> points;

	private:

	#if defined MRCV_POWER_SAVING || defined MRCV_SPEEDY
		cv::Mat points;
	#elif defined MRCV_DETAILED
		std::vector<cv::Mat> poses;
	#endif
};



}  // namespace
#endif
