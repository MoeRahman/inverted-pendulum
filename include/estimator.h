#pragma once

typedef struct{
    double x_est;         //position estimate         [m]
    double x_dot_est;     //velocity estimate         [m/s]
    double theta_est;     //pole angle estimate       [θ]
    double theta_dot_est; //angular velocity estimate [ω]
}estimated_state_t;

double angular_encoder_sensor(double theta_measurement);