
#ifndef MRCV_MAP_H
#define MRCV_MAP_H

#include <Frame.h>
#include <Kalman.h>

#define MRCV_DETAILED // hardcode for now

namespace MRCV {

typedef unsigned char uchar;


struct Observation {
	cv::Point2f pt;
};


struct Point {
	cv::Point3f pt;
	cv::Mat des;
	std::vector<Observation> observations;

	cv::Point3f global( Point& a) {
		// express point in global frame
	}
};

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
		bool contains( uchar a );
		void add( Point a );
		int size();

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
