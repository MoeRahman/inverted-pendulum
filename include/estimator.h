#pragma once

//Process Noise
#define POS_NOISE   1e-4
#define VEL_NOISE   1e-3
#define ANGLE_NOISE 1e-5
#define OMEGA_NOISE 1e-4

//Sensor Noise
#define POS_SENSOR_NOISE   1e-2
#define ANGLE_SENSOR_NOISE 1e-3

double angular_encoder_sensor(double theta_measurement);