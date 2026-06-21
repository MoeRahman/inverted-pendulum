#include "estimator.h"
#include "parameters.h"

/*
- Optimal Linear State Observer
- Kalman Filter
*/

double angular_encoder_sensor(double theta_measurement){
    return 0;
}

void kalman_filter(vect4d_t* state_estimate, vect4d_t* d_dt_state_estimate,double input, double measurement,void* params){
  
  double* kalman_gain = (double*)params;

  for(size_t i = 0; i < 4; ++i) {
      d_dt_state_estimate->arr[i] = 0.0;
  }

  double obsvr_error = measurement - state_estimate->arr[0];
  
  //d/dt x_est = A*x_est + B*u + Kf*(y - C*x_est)
  for(size_t i = 0; i < 4; ++i){
      for(size_t j = 0; j < 4; ++j){
          d_dt_state_estimate->arr[i] += A[i][j] * state_estimate->arr[j];
      }
      d_dt_state_estimate->arr[i] += B[i] * input + kalman_gain[i] * obsvr_error;
  }
}

void ekf(vect4d_t* state_estimate, vect4d_t* d_dt_state_estimate, double input, double measurement, void* params){

  //calculate d

}


double* set_estimator_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    {3.5499e-02,  6.4151e-01, -2.0772e+00, -1.1452e+01},
    {8.3155,     34.5735,   -103.5078,  -418.6079},
    {0,0,0,0}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}



