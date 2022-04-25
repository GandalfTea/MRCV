
#include <Kalman.h>

namespace MRCV {


Kalman::Kalman() {

	x.resize(12);
	for( size_t i = 0; i < 12; i++) {
		x[i] = 0;
	}	

	F.reshape(DIM, DIM) = Eigen::MatrixXd::Identity(DIM, DIM);
	
	// Update Position with respective velocity
	F[0][6] = this->delta_time;
	F[1][7] = this->delta_time;
	F[2][8] = this->delta_time;

	// Update rotation with respective angular velocity
	F[2][9]  = this->delta_time; 
	F[4][10] = this->delta_time;
	F[5][11] = this->delta_time;

}

void Kalman::predict() {

}


} // namespace
