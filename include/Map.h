
#ifndef MRCV_MAP_H
#define MRCV_MAP_H

#include <Frame.h>

namespace MRCV {


class Map {
	public:
		Map();
		~Map();

		// Update the map if there are any new points 
		void update( const Frame& frame );
		void clear();
		void draw();
		void video();

	private:
		std::vector<Frame> frames;
		cv::Mat points;
}



}  // namespace
#endif
