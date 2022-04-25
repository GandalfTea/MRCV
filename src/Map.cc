
#include <Map.h>

namespace MRCV {

Map::Map() {
	this->points = new std::vector<Point>;
	this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

Map::~Map() {}

void Map::update( cv::Mat pts, cv::Mat des ) {
	for( int i = 0; i < pts.cols; i++) {
		  	this->contains(des.row(i));
				Point a;
				cv::Mat temp = pts.col(i);
				a.pt = cv::Point3f( temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));
				a.des = des.row(i);
				this->add( a ); 
	}
}

bool Map::contains( cv::Mat des ) {
		std::vector<std::vector<cv::DMatch>> matches;
		matcher->knnMatch(des, this->des, matches, 2);
		//std::cout << matches[0][0].queryIdx << std::endl;
}

void Map::add ( Point a ) {
	this->points->push_back(a);
	cv::Mat desc = a.des;
	this->des.push_back(desc);
}

int Map::size() {
	return this->points->size();
}


}  // namespace
