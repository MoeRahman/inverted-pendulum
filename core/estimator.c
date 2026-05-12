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



