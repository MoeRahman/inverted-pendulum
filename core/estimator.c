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
    
    double g     = pendulum_params.g;
    double m     = pendulum_params.m;
    double M     = pendulum_params.M;
    double l     = pendulum_params.L;
    double b     = pendulum_params.b;
    double gamma = pendulum_params.gamma;

    double mt = M + m;
    double I  = (1.0 / 3.0) * m * l * l;

    double den = 4.0 * I * mt - l * l * m * m + l * l * m * mt;

    double a22 = (-4.0 * b * (I + (l * l * m) / 4.0)) / den;
    double a23 = (-g * l * l * m * m) / den;
    double a24 = (2.0 * gamma * l * m) / den;
    
    double a42 = (2.0 * b * l * m) / den;
    double a43 = (2.0 * g * l * m * mt) / den;
    double a44 = (-4.0 * gamma * mt) / den;

    double A[4][4] = {
        {0.0, 1.0, 0.0, 0.0},
        {0.0, a22, a23, a24},
        {0.0, 0.0, 0.0, 1.0},
        {0.0, a42, a43, a44}
    };

    double B[4] = {
        0.0,
        (4.0 * (I + (l * l * m) / 4.0)) / den,
        0.0,
        (-2.0 * l * m) / den
    };

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
    {12.435, 77.262, -171.370, -944.128}, //theta = 0
    {6.8046e-01, 2.2651e-01, -1.7617e-03, 5.0846e-05}, //theta = M_PI
    {6.8278e-03, 1.8618e-03, 1.2109e-05, 3.4978e-04}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}



