#pragma once

#include "types.h"
#include <math.h>
#include <stdbool.h>
#include <time.h>

#define ENABLE_DAMPING true
#define DISABLE_DAMPING false

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


extern const pendulum_params_t pendulum_params;
extern const motor_params_t motor_params;


void pendulum_dynamics(vect4d_t const *curr_state, 
                       vect4d_t  *next_state, 
                       const pendulum_params_t pendulum_params, 
                       const double Force,
                       const bool enable_damping);


void rk4_step(vect4d_t const *curr_state, 
              vect4d_t* next_state,
              const pendulum_params_t pendulum_parms, 
              const double F, const double dt, 
              const bool enable_damping);


double gaussian_generator(double mean, double std_dev);

