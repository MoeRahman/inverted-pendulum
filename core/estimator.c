#include "estimator.h"
#include "parameters.h"

/*
- Optimal Linear State Observer
- Kalman Filter
*/

double angular_encoder_sensor(double theta_measurement){
    return 0;
}

void kalman_filter(vect4d_t* state_estimate, 
                   vect4d_t* d_dt_state_estimate,
                   double input, 
                   double measurement,
                   void* params){

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


double* set_estimator_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    {8.3155,     34.5735,   -103.5078,  -418.6079},
    {0.011072,   0.061639,  -0.135092,  -0.744244},
    {6.8278e-03, 1.8618e-03, 1.2109e-05, 3.4978e-04}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}



