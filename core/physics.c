#include "physics.h"
#include <math.h>
/*
- Non-linear dynamics model for the inverted pendulum and DC motor
- ODE Solver
*/

pendulum_params_t pendulum_params = {.g = 9.80665, .m = 0.1, .M = 1, .L = 0.5};
motor_params_t motor_params = {.k1 = 1, .k2 = 1, .R  = 1, .r  = 1};

void pendulum_dynamics(const pendulum_state_t* curr_state, 
  pendulum_state_t* next_state, pendulum_params_t pendulum_params, double F){

    //pendulum state variables
    const double x          = curr_state->x;
    const double x_dot      = curr_state->x_dot;
    const double theta      = curr_state->theta;
    const double theta_dot  = curr_state->theta_dot;

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
    next_state->x = x_dot;
    next_state->x_dot = (a1 - a3)/a2;
    next_state->theta = theta_dot;
    next_state->theta_dot = (a4 - cos_theta*a1)/(L*a2);

  }


void rk4_step(const pendulum_state_t* curr_state, pendulum_state_t* next_state,
  pendulum_params_t pendulum_parms, const double F, const double dt){

    
  }

