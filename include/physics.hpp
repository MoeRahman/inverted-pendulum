#pragma once
#include <Eigen/Dense>

void nonlinear_dynamics(const Eigen::Vector4d* curr_state, Eigen::Vector4d* state_dot, 
  double F, double M, double m, double L, double g);

void rk4_step(const Eigen::Vector4d* curr_state, Eigen::Vector4d* next_state, 
  double F, double M, double m, double L, double g, double dt);




