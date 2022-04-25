
#ifndef MRCV_MAP_H
#define MRCV_MAP_H

#include <Frame.h>
#include <Kalman.h>

#define MRCV_DETAILED // hardcode for now

namespace MRCV {

typedef unsigned char uchar;

#if defined MRCV_POWER_SAVING || defined MRCV_SPEEDY
	#define MAP_MAX_SIZE 100000 * sizeof(Point)*4
#elif defined MRCV_DETAILED
	#define MAP_MAX_SIZE 1000000 * sizeof(double)*4
#endif


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
		bool contains( cv::Mat des );
		void add( Point a );
		int size();

		int mPointsFound = 0;
		cv::Ptr<cv::DescriptorMatcher> matcher; 
		cv::Mat des;

	private:

	#if defined MRCV_POWER_SAVING || defined MRCV_SPEEDY
		cv::Mat points;
	#elif defined MRCV_DETAILED
		std::vector<Point>* points;
		std::vector<cv::Mat> poses;
	#endif
};



}  // namespace
#endif
