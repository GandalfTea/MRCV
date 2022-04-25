

#if __has_include(<Eigen/Dense>)

#ifndef MRCV_KALMAN_FILTER_CORE
#define MRCV_KALMAN_FILTER_CORE

#include <cmath>
#include <Eigen/Dense>

namespace MRCV {

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdr;

struct Estimate {
	Eigen::VectorXd xk1;
	Eigen::VectorXd xk;
	MatrxXdr Pk1;
	MatrixXdr Pk;
	double t;
	std::vector<Eigen::VectorXd> y;
	std::vector<Eigen::VectorXd> z;
}; 

/*
Point used in the global Map
It contains the estimated coordinates (pt), all of the past observations and the credibility of the point (c)
The credibility updates when the point is updated, if the differences in observation are big, credibility is low
In order to easily do matrix dot products, we overloaded the * operator to only affect the pt       */

struct Point {
	cv::Point3f pt;
	std::vector<cv::Point3f> observations;
	float c;
};

Point operator*( Point& lhs, double rhs ) {
	lhs.pt.x *= rhs;
	lhs.pt.y *= rhs;
	lhs.pt.z *= rhs;	
	return lhs;
}

Point operator*( double lhs, Point* rhs ) {
	rhs.pt.x *= lhs;
	rhs.pt.y *= lhs;
	rhs.pt.z *= lhs;	
	return rhs;
}


/*
	Extended Kalman Filter with Symbolic Jacobian Matrices
	It is used to remove function and triangulatio noise to reduce drift
 
 	Components :
 		x 	: System state matrix, keeps track of entity pose and global map points
				: Posx, Posy, Posz, Rotx, Roty, Rotz, Velx, Vely, Velz, AVelx, AVely, AVelz, ...

	 	u		: Input matrix, used to keep track of outside influences on the entity
 W or Q	: Process noise matrix, covariance of the process noise w 
	 	K		: Kalman Gain Matrix
	 	F		: State transition matrix, defined the computation done on x for it to calibrate with new info
		P		: Covariance Matrix, estimation error
 G or H	: Input transition matrix, defines the computation done on new sensor data to be added to state	
		Z		: Sensor input on every cycle		
 		R		: Covariance of measurement noise
		
	Process :

		Covariances are assumed stationary:
		Q = COV( w, w.t() )
		R = COV( v, v.t() )
		P = COV( e, e.t() ) = COV( (x - x')(x - x').t() )


		1. Prediction : predict the next value based on last state, input variables and noise
		
				x10 = F * x00 + G * u + W
				x10 = (F*x00) + K*(y -C*x00) 
				P10 = A(I - KC)PA.t() + W

		2. Calibrate State : using the prediction and new sensor data, compute the new state
			
				K = P_k * H.t() * ( H * P_k * H.t() + R)^-1
				x11 = x10 + K * ( Z - h(x10) )

		3. Repeat

	Particularities : 
		* We will consider map points to be static and remain unchanged in the prediction step.
		* For now, we will consider no input variable
		* For now, we will consider no x-axis movement
	
	TODO:
		* Quanternion for rotation
		* Velocity and Acceleration                    */


const DIM = 12;	  // dimention of x
const EDIM = 12;  	// dimention of error
//const MEDIM = 0;  // dimention of main error


class Kalman {
	public:
		Kalman();
		Kalman( Eigen::vectorXd state_prediction, MatrixXdr cov_prediction );
		~Kalman();

		Estimate predict_and_update();

	protected:
		Eigen::VectorXd x;	
		MatrixXdr P;			

		void predict();
		void update ( double* in_x, double* in_P )	

	private:
		double filter_time;
		MatrixXdr W;		
		MatrixXdr K;
		MatrixXdr F;
		Eigen::vectorXd p;	// error variance
		Eigen::vectorXd r;	// measuring uncertenty 

};


#ifdef MAP_POINT_KALMAN

struct Prediction1D {
	float state;
	float prediction;
};

template <typename T>
class Kalman1D {
	public:
		Kalman1D(T prediction, float q, float p, float z) 
			: state(prediction), prediction(0), q(q), p(p), z(z), r(pow(2, z)) {}
		~Kalman1D();

		Prediction1D update_and_predict( T data ) {

			// Predict
			prediction = state;
			p += q;


			// Update State
			float K = p / ( p + r);
			state += K * ( data - state);
			p = float(1 - K) * p;

			// Return
			Prediction1D ret;
			ret.state = state;
			ret.prediction = prediction;
			return ret;
		}

	private:
		T state;
		T prediction;
		float q, p, z, r;
};

#endif




} // namespace
#endif
#endif
