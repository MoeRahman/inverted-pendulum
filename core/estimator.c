#include "estimator.h"
/*
- Optimal Linear State Observer
- Kalman Filter
*/

double angular_encoder_sensor(double theta_measurement){
    return 0;
}

void observation_update(vect4d_t *measurement, vect4d_t *state, vect4d_t *sensor_covar, double dt){
    // measurement of the position and angle states are the only ones that are observable

    measurement->state.x = state->state.x + sensor_covar->state.x;
    measurement->state.theta = state->state.theta + sensor_covar->state.theta;

}

vect4d_t kalman_filter(vect4d_t *state, vect4d_t *process_covar, vect4d_t *sensor_covar){

    vect4d_t output;
    
    return output;
}


const double *set_estimator_gain(gain_t gain){

  static const double GAIN_TABLE[3][4] = {
    {-7.0711e+03,  -3.9489e+03,  -1.1089e+04,  -2.3468e+03},
    {-1000,   -735.01,  -2879.72,   -598.14},
    {-3162.3,  -1914.9,  -5952.5,  -1251.1}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}



