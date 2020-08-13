/*
 * This Kalman filter is implemented as a learning exercise.
 * It is used in the final version of the cube, but could be 
 * almost drag and drop replaced by a more efficient library.
 * 
 * Uses a custom Matrix<T> class to perform matrix operations
 * in an arduino friendly (space saving) way.
 */

#include "kalman.hpp"
#include <algorithm>
#include <functional>

Kalman_Filter::Kalman_Filter(
  Matrix<float> &_cov_mat,     // Covariance Matrix. //[0.5,0,0,0.01]
  Matrix<float> &_trans_mat,   // State transition Matrix
  Matrix<float> &_Q,           // Process noise// Time Step
  Matrix<float> &_R,           // Measurement Noise
  Matrix<float> &_x_hat,       // State vector. 2 states in this case, MPU roll, and MPU drift [ROLL,DRIFT]
  Matrix<float> &_kal_gain,    // Kalman Gain, [0,0].
  Matrix<float> &_psi,

  float _dt)                  // Timestep
: cov_mat(_cov_mat), trans_mat(_trans_mat), Q(_Q), R(_R), x_hat(_x_hat), kal_gain(_kal_gain), psi(_psi), dt(_dt)  
{
  H = Matrix<float>
  I = Matrix<float> {H.GetRows(),H.GetRows(),0};
}

//initialize the kalman filter with time 0 and the 
//current states at time 0.
void Kalman_Filter::init(float t0 , Matrix<float>&x0) {
  this->t=t0;
  this->x_hat = x0;
}

void Kalman_Filter::update(const int gyro_deg, const long acc_roll) {
  //Calculate zk and uk
  Matrix<float> uk = {gyro_deg / 65.5};   //gyro roll rate per second
  Matrix<float> zk {acc_roll};  //accel angle measurement in degrees

  //Predict States using a priori state estimation equation.
  Matrix<float> xhat_minus = trans_mat * x_hat + psi * uk;
  Matrix<float> cov_mat_minus = trans_mat * cov_mat * trans_mat.Transpose() + Q;

  //Calculate Kalman Gain
  Matrix<float> S = H * cov_mat_minus * H.Transpose() + R;
  Matrix<float> K = cov_mat_minus * H.Transpose() * S.Invert();

  //Update States based on measurements
  x_hat = xhat_minus + K * (zk - (H * xhat_minus));
  cov_mat = (I - K * H) * cov_mat_minus; 
}
