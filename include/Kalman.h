
#ifndef MRCV_KALMAN_FILTER_CORE
#define MRCV_KALMAN_FILTER_CORE

#include <cmath>

namespace MRCV {

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

			// Update State
			float K = p / ( p + r);
			state += K * ( data - state);
			p = float(1 - K) * p;

			// Predict
			prediction = state;
			p += q;

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




} // namespace
#endif
