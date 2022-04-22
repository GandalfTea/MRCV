
#include <Map.h>

namespace MRCV {

Map::Map() {
	this->points = new std::vector<Point>;
}

Map::~Map() {}

void Map::update( cv::Mat pts, cv::Mat des ) {
	for( int i = 0; i < pts.cols; i++) {
		//if( !this->contains(des.row(i))) {
			Point a;
			cv::Mat temp = pts.col(i);
			a.pt = cv::Point3f( temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));
			a.des = des.row(i);
			this->add( a ); 
		//}
	}
}

bool Map::contains( uchar a ) {}

void Map::add ( Point a ) {
	this->points->push_back(a);
}

int Map::size() {
	return this->points->size();
}


}  // namespace
