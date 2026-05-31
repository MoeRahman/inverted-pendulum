#pragma once

#include "types.h"
#include "stddef.h"

//Process Noise
#define POS_NOISE   1e-4
#define VEL_NOISE   1e-3
#define ANGLE_NOISE 1e-5
#define OMEGA_NOISE 1e-4

//Sensor Noise
#define POS_SENSOR_NOISE   1e-3
#define ANGLE_SENSOR_NOISE 1e-3

double angular_encoder_sensor(double theta_measurement);

void kalman_filter(vect4d_t* state_estimate,
                   vect4d_t* d_dt_state_estimate,
                   double input, 
                   double measurement,
                   void* params);

double* set_estimator_gain(gain_t gain);