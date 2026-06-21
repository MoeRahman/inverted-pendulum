#pragma once

#include "types.h"
#include "stddef.h"

//Process Noise
#define POS_NOISE   0
#define VEL_NOISE   1e-7
#define ANGLE_NOISE 0
#define OMEGA_NOISE 1e-6
//Noise acting on the input rather than each state(this ensures physical laws are obeyed)
#define PROCESS_NOISE_MEAN  0
#define PROCESS_NOISE_COVAR 1e-3

//Sensor Noise
#define POS_SENSOR_MEAN    0
#define POS_SENSOR_COVAR   1e-2
#define ANGLE_SENSOR_NOISE 1e-3

double angular_encoder_sensor(double theta_measurement);

void kalman_filter(vect4d_t* state_estimate,
                   vect4d_t* d_dt_state_estimate,
                   double input, 
                   double measurement,
                   void* params);

void ekf(vect4d_t* state_estimate,
        vect4d_t* d_dt_state_estimate,
        double input,
        double measurement,
        void* params);

double* set_estimator_gain(gain_t gain);