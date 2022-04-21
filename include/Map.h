
#ifndef MRCV_MAP_H
#define MRCV_MAP_H

#include <Frame.h>
#include <Kalman.h>

namespace MRCV {


typedef unsigned char uchar;



struct Observation {
	cv::Point2f pt;
};


struct Point {
	cv::Point2f pt;
	uchar des;
	std::vector<Observation> observations;
};



class Map {
	public:
		Map();
		~Map();

		void update( const Frame& frame );
		void clear();
		void draw();
		void video();

	private:
		std::vector<Point> points;
		std::vector<cv::Mat> poses;
};



}  // namespace
#endif
