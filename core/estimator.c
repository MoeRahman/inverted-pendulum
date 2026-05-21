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

vect4d_t kalman_filter(vect4d_t *state_estimate, vect4d_t *kalman_gain,
                       double input, double measurement){

    vect4d_t d_dt_state_estimate = {0, 0, 0, 0};
    
    double a22 = pendulum_params.b/pendulum_params.M;
    double a23 = (pendulum_params.m*pendulum_params.g)/pendulum_params.M;
    double a24 = pendulum_params.gamma/pendulum_params.M;
    double a42 = pendulum_params.b/(pendulum_params.M*pendulum_params.L);
    double a43 = ((pendulum_params.m + pendulum_params.M)/(pendulum_params.M*pendulum_params.L))*pendulum_params.g;
    double a44 = a43*((pendulum_params.gamma)/(pendulum_params.m*pendulum_params.L*pendulum_params.g));

    //State Transition Matrix
    double A[4][4] = {
        {0,    1,    0,    0},
        {0, -a22, -a23,  a24},
        {0,    0,    0,    1},
        {0,  a42,  a43, -a44}
    };

    //Input Matrix
    double B[4] = {0, 1/pendulum_params.M, 0, -1/(pendulum_params.M*pendulum_params.L)};

    //Measurement Matrix
    double C[4] = {1, 0, 0, 0};

    // d/dt x_est = A*x_est + B*u + Kf*(y - C*x_est)
    for(size_t i = 0; i < 4; ++i){
      for(size_t j = 0; j < 4; ++j){
        d_dt_state_estimate.arr[i] += A[i][j]*state_estimate->arr[j];
      }
      d_dt_state_estimate.arr[i] +=  B[i]*input + kalman_gain->arr[i]*(measurement - C[0]*state_estimate->state.x);
    }

    return d_dt_state_estimate;
}


vect4d_t set_estimator_gain(gain_t gain){

  static const double GAIN_TABLE[3][4] = {
    {10.236, 52.391, -58.799, -300.807},
    {1, 2, 3, 4},
    {4, 3, 2, 1}
  };

  vect4d_t kalman_gains = {0,0,0,0};

  if((gain >= 0) && (gain < 3)){
    memcpy(kalman_gains.arr, GAIN_TABLE[gain], sizeof(kalman_gains.arr));
  }

  return kalman_gains;
}



