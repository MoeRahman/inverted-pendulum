#include "physics.hpp"

void nonlinear_dynamics(const Eigen::Vector4d* curr_state, Eigen::Vector4d* state_dot, double F, double M, double m, double L, double g){
  
  double cart_x = (*curr_state)(0);
  double cart_vel = (*curr_state)(1);
  double theta = (*curr_state)(2);
  double theta_dot = (*curr_state)(3);

  double a1 = F + m*L*theta_dot*theta_dot*std::sin(theta);
  double a2 = M + m*std::sin(theta)*std::sin(theta);
  double a3 = m*g*std::sin(theta)*std::cos(theta);
  double a4 = (M + m)*g*std::sin(theta);
  
  (*state_dot)(0) = cart_vel;
  (*state_dot)(1) = (a1 - a3)/a2;
  (*state_dot)(2) = theta_dot;
  (*state_dot)(3) = (a4 - std::cos(theta)*a1)/(L*a2);
}

void rk4_step(const Eigen::Vector4d* curr_state, Eigen::Vector4d* next_state, double F, double M, double m, double L, double g, double dt){

  Eigen::Vector4d k1 = Eigen::Vector4d::Zero();
  Eigen::Vector4d k2 = Eigen::Vector4d::Zero();
  Eigen::Vector4d k3 = Eigen::Vector4d::Zero();
  Eigen::Vector4d k4 = Eigen::Vector4d::Zero();

  Eigen::Vector4d state = (*curr_state);
  
  nonlinear_dynamics(&state, &k1, F, M, m, L, g);
  state = (*curr_state) + 0.5*dt*k1;
  nonlinear_dynamics(&state, &k2, F, M, m, L, g);
  state = (*curr_state) + 0.5*dt*k2 ;
  nonlinear_dynamics(&state, &k3, F, M, m, L, g);
  state = (*curr_state) + dt*k3;
  nonlinear_dynamics(&state, &k4, F, M, m, L, g);

  (*next_state) = (*curr_state) + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);

}