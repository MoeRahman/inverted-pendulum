#pragma once

#include "parameters.h"
#include "types.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#define ENABLE_DAMPING true
#define DISABLE_DAMPING false

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void pendulum_dynamics(vect4d_t* curr_state, 
                       vect4d_t* next_state, 
                       double input,
                       double measurement,
                       void* params);


void rk4_step(dynamics_func sys_func,
              vect4d_t* curr_state, 
              vect4d_t* next_state,
              double input, 
              double measurement,
              void* params,
              const double dt);


double gaussian_generator(double mean, double std_dev);

