#include "physics.h"

/*
- Non-linear dynamics model for the inverted pendulum and DC motor
- ODE Solver
*/

double gaussian_generator(double mean, double std_dev){

  // Generate two uniform random numbers between 0 and 1
  double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0); 
  double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);

  // Box-Muller Transform
  double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);

  // Scale by standard deviation and shift by mean
  return (z0 * std_dev + mean);
}

void pendulum_dynamics(vect4d_t* curr_state, 
                       vect4d_t* next_state, 
                       double input, 
                       double measurement,
                       void* params){

    if (curr_state == NULL || next_state == NULL) {
        fprintf(stderr, "[%s error] Null pointer passed for state parameters.\n", __func__);
        return;
    }

    //pendulum state variables
    double x          = curr_state->state.x;
    double x_dot      = curr_state->state.x_dot;
    double theta      = curr_state->state.theta;
    double theta_dot  = curr_state->state.theta_dot;

    //pendulum params
    double g = pendulum_params.g;
    double m = pendulum_params.m;
    double M = pendulum_params.M;
    double L = pendulum_params.L;

    //damping constants
    double b = pendulum_params.b;
    double gamma = pendulum_params.gamma;

    //trig ratios [angle in radians]
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    //non-linear dynamics
    double a0 = (gamma*theta_dot)/(m*L);
    double a1 = input - b*x_dot + m*L*theta_dot*theta_dot*sin_theta;
    double a2 = M + m*sin_theta*sin_theta;
    double a3 = m*g*sin_theta*cos_theta;
    double a4 = (M + m)*(g*sin_theta - a0);

    //next state
    *next_state = (vect4d_t){x_dot, (a1 - a3)/a2 + (m*a0*cos_theta), theta_dot, (a4 - cos_theta*a1)/(L*a2)};
}

void rk4_step(dynamics_func sys_func,
              vect4d_t* curr_state, vect4d_t* next_state,
              double input, double measurement, void* params, const double dt){

    if (curr_state == NULL || next_state == NULL || sys_func == NULL) {
        fprintf(stderr, "[%s error] Null pointer passed for state parameters.\n", __func__);
        return;
    }

    vect4d_t k1 = {0}, k2 = {0}, k3 = {0}, k4 = {0};
    vect4d_t state = *curr_state;
    vect4d_t *k[] = {&k1, &k2, &k3};

    for(size_t j = 0; j < 3; ++j){
      sys_func(&state, k[j], input, measurement, params);
      //pendulum_dynamics(&state, k[j], input, 0);
      
      for(size_t i = 0; i < 4; ++i){
        state.arr[i] = curr_state->arr[i] + 0.5*dt*(k[j]->arr[i]);
      }
    }

    sys_func(&state, &k4, input, measurement, params);
    //pendulum_dynamics(&state, &k4, input, 0);
    for(size_t i = 0; i < 4; ++i){
      next_state->arr[i] = curr_state->arr[i] + (dt/6.0)*(k1.arr[i] + 2.0*k2.arr[i] + 2.0*k3.arr[i] + k4.arr[i]);
    }
}