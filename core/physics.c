#include "physics.h"

/*
- Non-linear dynamics model for the inverted pendulum and DC motor
- ODE Solver
*/

pendulum_params_t pendulum_params = {.g = 9.80665, .m = 0.1, .M = 1, .l = 0.5};
motor_params_t motor_params = {.k1 = 1, .k2 = 1, .R  = 1, .r  = 1};

void pendulum_dynamics(const pendulum_state* curr_state, 
  pendulum_state* next_state, pendulum_params_t pendulum_params, double F){}


void rk4_step(const pendulum_state* curr_state, pendulum_state* next_state,
  pendulum_params_t pendulum_parms, const double F, const double dt){}

