#pragma once

typedef struct{
  const double g; //gravity
  const double m; //ball mass
  const double M; //cart mass
  const double l; //pole length
}pendulum_params_t;

typedef struct{
  const double k1; //torque constant
  const double k2; //back-emf constant
  const double R;  //motor internal resistance
  const double r;  //torque to force ratio
}motor_params_t;

typedef struct{
  double x;
  double x_dot;
  double theta;
  double theta_dot;
}pendulum_state;

extern pendulum_params_t pendulum_params;
extern motor_params_t motor_params;

void pendulum_dynamics(const pendulum_state* curr_state, 
  pendulum_state* next_state, pendulum_params_t pendulum_params, double F);

void rk4_step(const pendulum_state* curr_state, pendulum_state* next_state,
  pendulum_params_t pendulum_parms, const double F, const double dt);