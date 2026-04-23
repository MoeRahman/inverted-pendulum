
#include "physics.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
- Non-linear dynamics model for the inverted pendulum and DC motor
- ODE Solver
*/

const pendulum_params_t pendulum_params = {.g = 9.80665, .m = 0.1, .M = 1, .L = 0.5};
const motor_params_t motor_params = {.k1 = 1, .k2 = 1, .R  = 1, .r  = 1};

double guassian_generator(double mean, double std_dev){

  srand(time(NULL));

  // Generate two uniform random numbers between 0 and 1
  double u1 = (double)rand() / RAND_MAX;
  double u2 = (double)rand() / RAND_MAX;

  // Box-Muller Transform
  double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);

  // Scale by standard deviation and shift by mean
  return (z0 * std_dev + mean);
}

void pendulum_dynamics(state_t const *curr_state, 
                       state_t* next_state, 
                       const pendulum_params_t pendulum_params, 
                       const double F){

    if (curr_state == NULL || next_state == NULL) {
        fprintf(stderr, "[%s error] Null pointer passed for state parameters.\n", __func__);
        return;
    }

    //Gaussian Noise 
    const double x1_noise = guassian_generator(0, 0.25);
    const double x2_noise = guassian_generator(0, 0.025);
    const double x3_noise = guassian_generator(0, 0.004);
    const double x4_noise = guassian_generator(0, 0.0001);

    //pendulum state variables
    const double x          = x1_noise + curr_state->pendulum.x;
    const double x_dot      = x2_noise + curr_state->pendulum.x_dot;
    const double theta      = x3_noise + curr_state->pendulum.theta;
    const double theta_dot  = x4_noise + curr_state->pendulum.theta_dot;

    //pendulum params
    const double g = pendulum_params.g;
    const double m = pendulum_params.m;
    const double M = pendulum_params.M;
    const double L = pendulum_params.L;

    //trig ratios
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    //non-linear dynamics
    double a1 = F + m*L*theta_dot*theta_dot*sin_theta;
    double a2 = M + m*sin_theta*sin_theta;
    double a3 = m*g*sin_theta*cos_theta;
    double a4 = (M + m)*g*sin_theta;

    //next state
    *next_state = (state_t){x_dot, (a1 - a3)/a2, theta_dot, (a4 - cos_theta*a1)/(L*a2)};

}

void rk4_step(state_t const *curr_state, 
              state_t* next_state,
              pendulum_params_t pendulum_parms, 
              const double F, 
              const double dt){

    if (curr_state == NULL || next_state == NULL) {
        fprintf(stderr, "[%s error] Null pointer passed for state parameters.\n", __func__);
        return;
    }

    state_t k1 = {0, 0, 0, 0};
    state_t k2 = {0, 0, 0, 0}; 
    state_t k3 = {0, 0, 0, 0};
    state_t k4 = {0, 0, 0, 0};

    state_t state = *curr_state;

    state_t *k[] = {&k1, &k2, &k3};

    for(size_t j = 0; j < 3; ++j){
      pendulum_dynamics(&state, k[j], pendulum_params, F);
      
      for(size_t i = 0; i < 4; ++i){
        state.arr[i] = curr_state->arr[i] + 0.5*dt*(k[j]->arr[i]);
      }
    }

    pendulum_dynamics(&state, &k4, pendulum_params, F);
    for(size_t i = 0; i < 4; ++i){
      next_state->arr[i] = curr_state->arr[i] + (dt/6.0)*(k1.arr[i] + 2.0*k2.arr[i] + 2.0*k3.arr[i] + k4.arr[i]);
    }
}