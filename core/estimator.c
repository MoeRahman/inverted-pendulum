#include "estimator.h"
/*
- Optimal Linear State Observer
- Kalman Filter
*/

double angular_encoder_sensor(double theta_measurement){
    return 0;
}

vect4d_t observation_update(vect4d_t *state, vect4d_t *sensor_covar){

    vect4d_t output;

    for(size_t i = 0; i < 4; ++i){
        state->arr[i] += sensor_covar->arr[i];
    }

    return output;
}

vect4d_t kalman_filter(vect4d_t *state, vect4d_t *process_covar, vect4d_t *sensor_covar){

    vect4d_t output;
    
    return output;
}



