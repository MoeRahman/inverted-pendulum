#pragma once

#include "types.h"

//Process Noise
#define POS_NOISE   1e-4
#define VEL_NOISE   1e-3
#define ANGLE_NOISE 1e-5
#define OMEGA_NOISE 1e-4

//Sensor Noise
#define POS_SENSOR_NOISE   1e-2
#define ANGLE_SENSOR_NOISE 1e-3

double angular_encoder_sensor(double theta_measurement);

vect4d_t observation_update(vect4d_t state, vect4d_t sensor_covar);

vect4d_t kalman_filter(vect4d_t state, vect4d_t process_covar, vect4d_t sensor_covar);
