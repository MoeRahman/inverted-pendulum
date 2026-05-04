
#include "physics.h"
#include <stdlib.h>
#include <stdio.h>

/*
- Non-linear dynamics model for the inverted pendulum and DC motor
- ODE Solver
*/

const pendulum_params_t pendulum_params = {.G = 9.80665, .m = 10, .M = 20, .L = 0.5, .b = 2, .g = 1};
const motor_params_t motor_params = {.k1 = 1, .k2 = 1, .R  = 1, .r  = 1};

double gaussian_generator(double mean, double std_dev){

  // Generate two uniform random numbers between 0 and 1
  double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0); 
  double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);

  // Box-Muller Transform
  double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);

  // Scale by standard deviation and shift by mean
  return (z0 * std_dev + mean);
}

void pendulum_dynamics(vect4d_t const *curr_state, vect4d_t* next_state, 
  const pendulum_params_t pendulum_params, const double F, const bool enable_damping){

    if (curr_state == NULL || next_state == NULL) {
        fprintf(stderr, "[%s error] Null pointer passed for state parameters.\n", __func__);
        return;
    }

    //pendulum state variables
    const double x          = curr_state->pendulum.x;
    const double x_dot      = curr_state->pendulum.x_dot;
    const double theta      = curr_state->pendulum.theta;
    const double theta_dot  = curr_state->pendulum.theta_dot;

    //pendulum params
    const double G = pendulum_params.G;
    const double m = pendulum_params.m;
    const double M = pendulum_params.M;
    const double L = pendulum_params.L;

    //damping constants
    double b = 0;
    double g = 0;

    if(enable_damping){
      b = pendulum_params.b;
      g = pendulum_params.g;
    }

    //trig ratios [angle in radians]
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    //non-linear dynamics
    double a0 = (g*theta_dot)/(m*L);
    double a1 = F - b*x_dot + m*L*theta_dot*theta_dot*sin_theta;
    double a2 = M + m*sin_theta*sin_theta;
    double a3 = m*G*sin_theta*cos_theta;
    double a4 = (M + m)*(G*sin_theta - a0);

    //next state
    *next_state = (vect4d_t){x_dot, (a1 - a3)/a2 + (m*a0*cos_theta), theta_dot, (a4 - cos_theta*a1)/(L*a2)};
}

void rk4_step(vect4d_t const *curr_state, vect4d_t* next_state,
              pendulum_params_t pendulum_parms, 
              const double F, const double dt,
              const bool enable_damping){

    if (curr_state == NULL || next_state == NULL) {
        fprintf(stderr, "[%s error] Null pointer passed for state parameters.\n", __func__);
        return;
    }

    vect4d_t k1 = {0}, k2 = {0}, k3 = {0}, k4 = {0};
    vect4d_t state = *curr_state;
    vect4d_t *k[] = {&k1, &k2, &k3};

    for(size_t j = 0; j < 3; ++j){
      pendulum_dynamics(&state, k[j], pendulum_params, F, enable_damping);
      
      for(size_t i = 0; i < 4; ++i){
        state.arr[i] = curr_state->arr[i] + 0.5*dt*(k[j]->arr[i]);
      }
    }

    pendulum_dynamics(&state, &k4, pendulum_params, F, enable_damping);
    for(size_t i = 0; i < 4; ++i){
      next_state->arr[i] = curr_state->arr[i] + 
      (dt/6.0)*(k1.arr[i] + 2.0*k2.arr[i] + 2.0*k3.arr[i] + k4.arr[i]);
    }
}