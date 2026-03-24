#include "physics.hpp"
#include <cmath>
#include <iostream>
#include <fstream>


void nonlinear_dynamics(const Eigen::Vector4f* curr_state, Eigen::Vector4f* state_dot, 
  double F, double M, double m, double L, double g);

void rk4_step(const Eigen::Vector4f* curr_state, Eigen::Vector4f* next_state, 
  double F, double M, double m, double L, double g, double dt);


int main() {

  constexpr double g = 9.80665f;
  constexpr double m = .1f; //ball mass
  constexpr double M = 1.f; //cart mass
  constexpr double l = .5f; //pole length

  //write data to csv file
  std::ofstream myFile("data.csv");

  if (!myFile.is_open()) {
    std::cerr << "Error: Could not open the file!" << std::endl;
    return 1;
  }

  myFile << "Time,Pos_X,Angle\n";

  //Simulate
  const double duration {5.f};
  double time{0.f};
  double dt{0.001f};

  Eigen::Matrix4f A;
  Eigen::Vector4f x;
  Eigen::Vector4f x_dot;
  Eigen::Vector4f B;
  Eigen::RowVector4f K;
  double u;
  Eigen::Vector4f setpoint;

  // This is the linearized state
  A << 0.f, 1.f, 0.f, 0.f, 
       0.f, 0.f, -m*g/M, 0.f, 
       0.f, 0.f, 0.f, 1.f, 
       0.f, 0.f, (M + m)*g/(M*l), 0.f;
  x << 0.f, 0.f, .05f, 0.f;
  x_dot << 1.f, 0.f, 0.f, 0.f;
  B << 0.f, 1.f/M, 0.f, -1.f/(M*l);

  //K << -1.3003f,-2.3173f,-26.6124f,-5.6587f;
  K << -874.92f,-307.24f,-843.75f,-176.62f;
  //K << -12.237f,-20.598f,-110.906f,-22.799f;
  //K << -0.1225,-0.3952,-15.5213,-2.6976;
  setpoint << 1.f, 0.f, 0.f, 0.f;

  double curr_state[4] = {0, 0, 0, 0};
  double state_dot[4] = {0, 0, 0, 0};

  size_t count = 0;

  while(time < duration){

    // if((count%5000) == 0){
    //   setpoint(0) *= -1;
    // }

    myFile << time << "," << x(0) << "," << x(2) << "\n";

    //u = -(K * (x - setpoint))(0);
    u = (x - setpoint)(0);

    nonlinear_dynamics(&x, &x_dot, u, M, m, l, g);
    rk4_step();
    time += dt;
    count += 1;

  }

  myFile.close();

  return 0;
}

void nonlinear_dynamics(const Eigen::Vector4f* curr_state, Eigen::Vector4f* state_dot, double F, double M, double m, double L, double g){
  
  double a1 = F + m*L*(*curr_state)(2)*(*curr_state)(2)*std::sin((*curr_state)(1));
  double a2 = M + m*std::sin((*curr_state)(1))*std::sin((*curr_state)(1));
  double a3 = m*g*std::sin((*curr_state)(1))*std::cos((*curr_state)(1));
  double a4 = (M + m)*g*std::sin((*curr_state)(1));
  
  (*state_dot)(0) = (*curr_state)(1);
  (*state_dot)(1) = (a1 - a3)/a2;
  (*state_dot)(2) = (*curr_state)(3);
  (*state_dot)(3) = (a4 - std::cos((*curr_state)(1))*a1)/(L*a2);
}

void rk4_step(const Eigen::Vector4f* curr_state, Eigen::Vector4f* next_state, double F, double M, double m, double L, double g, double dt){

  
  

}