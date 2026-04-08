#pragma once
#include <Eigen/Dense>

typedef struct{
  const double g; //gravity
  const double m; //ball mass
  const double M; //cart mass
  const double l; //pole length
}pendulumParams_t;

typedef struct{
  const double k1; //torque constant
  const double k2; //back-emf constant
  const double R;  //motor internal resistance
  const double r;  //torque to force ratio
}dcMotorParams_t;

void nonlinear_dynamics(const Eigen::Vector4d* curr_state, Eigen::Vector4d* state_dot, 
  double F, double M, double m, double L, double g);

void rk4_step(const Eigen::Vector4d* curr_state, Eigen::Vector4d* next_state, 
  double F, double M, double m, double L, double g, double dt);




