
#include <Map.h>

namespace MRCV {

Map::Map() {
	this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

Map::~Map() {}

void Map::update( cv::Mat pts, cv::Mat des ) {

	mPointsFound = 0;
	mNewPoints = 0;
	mRepeatingPoints = 0;

	/*
	// Delete repeating points in input
	for( size_t i = 0; i <= pts.cols; i++) {
		float x = pts.at<float>(i, 0); 
		float y = pts.at<float>(i, 1); 
		float z = pts.at<float>(i, 2); 

		auto point = cv::Point3f(x, y, z);
	
		bool exists = false;
		for( auto i : points) {
			if( i.x == point.x && i.y == point.y && i.z == point.z) {
				iRepeatingPoints++;
				exists = true;
				continue;
			}
		}
	}
	*/

	// Update existing map points
	std::vector<int> delPoints;
	auto matches = this->contains(des);
	if( matches.size() != 0 ){
		for( auto i : matches ) {
			if( i[0].queryIdx < pts.cols ) {

				float zX = pts.at<float> ( i[0].queryIdx, 0 );
				float zY = pts.at<float> ( i[0].queryIdx, 1 );
				float zZ = pts.at<float> ( i[0].queryIdx, 2 );

				// very shitty mean
				auto updated = this->points[i[0].trainIdx].pt;
				updated.x = (updated.x + zX) / 2;
				updated.y = (updated.y + zY) / 2;
				updated.z = (updated.z + zZ) / 2;
				this->points[i[0].trainIdx].pt = updated;
				
				delPoints.push_back(i[0].trainIdx);
				mPointsFound++;
			}
		}
	}

	// Create new map point
	for( int i = 0; i < pts.cols; i++) {
		if( std::find(delPoints.begin(), delPoints.end(), i) == delPoints.end() ) {
			Point a;
			a.pt = cv::Point3f( pts.at<float>(i,0), pts.at<float>(i,1), pts.at<float>(i,2));
			a.des = des.row(i);
			this->add( a ); 
		}
	}
}


std::vector<std::vector<cv::DMatch>> Map::contains( cv::Mat des ) {
	std::vector<std::vector<cv::DMatch>> matches;
	if(this->des.rows != 0) {
		matcher->knnMatch(des, this->des, matches, 2);
	}
	return matches;
}



void Map::add ( Point a ) {
	for( auto i : points) {
		if(i.pt == a.pt) {
			mRepeatingPoints++;
			return;
		}
	}
	this->points.push_back(a);
	this->des.push_back(a.des);
	mNewPoints++;
}



int Map::size() {
	return this->points.size();
}


}  // namespace
