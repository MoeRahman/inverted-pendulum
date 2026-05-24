#include "estimator.h"
#include "parameters.h"
#include <string.h>
#include <stdio.h>
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
    
    double a22 = pendulum_params.b/pendulum_params.M;
    double a23 = (pendulum_params.m*pendulum_params.g)/pendulum_params.M;
    double a24 = pendulum_params.gamma/pendulum_params.M;
    double a42 = pendulum_params.b/(pendulum_params.M*pendulum_params.L);
    double a43 = ((pendulum_params.m + pendulum_params.M)/(pendulum_params.M*pendulum_params.L))*pendulum_params.g;
    double a44 = a43*((pendulum_params.gamma)/(pendulum_params.m*pendulum_params.L*pendulum_params.g));

    //State Transition Matrix
    double A[4][4] = {
        {0,    1,    0,    0},
        {0, -a22,  a23, -a24},
        {0,    0,    0,    1},
        {0, -a42, -a43, -a44}
    };

    //Input Matrix
    double B[4] = {0, 1/pendulum_params.M, 0, -1/(pendulum_params.M*pendulum_params.L)};

    //Measurement Matrix
    double C[4] = {1, 0, 0q, 0};
    double obsvr_error = measurement - C[0]*state_estimate->state.x;

    // d/dt x_est = A*x_est + B*u + Kf*(y - C*x_est)
    for(size_t i = 0; i < 4; ++i){
      for(size_t j = 0; j < 4; ++j){
        d_dt_state_estimate->arr[i] += A[i][j]*state_estimate->arr[j];
      }
      d_dt_state_estimate->arr[i] +=  B[i]*input + kalman_gain[i]*obsvr_error;
    }
}


double* set_estimator_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    {10.236, 52.391, -58.799, -300.807}, //theta = 0
    {6.8046e-01, 2.2651e-01, -1.7617e-03, 5.0846e-05}, //theta = M_PI
    {6.8278e-03, 1.8618e-03, 1.2109e-05, 3.4978e-04}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}



