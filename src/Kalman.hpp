/* 
 * A simple kalman filter implementation.
 * Implemented as a learning exercise, uses a custom Matrix<T> class
 * as Eigen is not available on arduino.
 *
*/ 

#ifndef kalman_HPP
#define kalman_HPP
#include Matrix.hpp

class Kalman_Filter(){

//Q - Process noise Covariance
//R - Measurement noise Covariance
//trans_mat - State transition Matrix, propagates the state forward. ex: [d1,1,0,dt]
//H - A matrix with width the number of state vars, and height number of measurement vars.
//kal_gain - Gain/Blending factor (minimizes the a posteriori error covariance)
//phi - Control transition matrix

//x_hat = a posteriori state estimate 

public:
Kalman_Filter(
	  double dt,
	  const Matrix<float>&cov_mat,
	  const Matrix<float>&trans_mat,
	  const Matrix<float>&phi,
	  const Matrix<float>&Q,
	  const Matrix<float>&R,
	);
	Kalman_Filter();
	//Initialize filter with time zero, and the state of the system at t0
	void init(float t0, const Matrix<float>&x0);

	// Updates the filter upon recieving new measurements
	void update(const Matrix<float>&measurements);

private:
	Matrix<float> kal_gain;
	Matrix<float> H;

	Matrix<float>&x_hat,
	Matrix<float>&cov_mat,
	Matrix<float>&trans_mat,
	Matrix<float>&phi,
	Matrix<float>&Q,
	Matrix<float>&R,

	float dt,t0,t;
};

#endif
