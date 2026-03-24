#include "physics.hpp"
#include <iostream>
#include <fstream>

int main() {

  constexpr float g = 9.80665f;
  constexpr float m = .1f; //ball mass
  constexpr float M = 1.f; //cart mass
  constexpr float l = .5f; //pole length

  //write data to csv file
  std::ofstream myFile("data.csv");

  if (!myFile.is_open()) {
    std::cerr << "Error: Could not open the file!" << std::endl;
    return 1;
  }

  myFile << "Time,Pos_X,Angle\n";

  //Simulate
  const float duration {10.f};
  float time{0.f};
  float dt{0.001f};

  Eigen::Matrix4f A;
  Eigen::Vector4f x;
  Eigen::Vector4f x_dot;
  Eigen::Vector4f B;
  Eigen::RowVector4f K;
  float u;
  Eigen::Vector4f setpoint;

  // This is the linearized state
  // TODO: Need Non-Linear Model
  A << 0.f, 1.f, 0.f, 0.f, 
       0.f, 0.f, -m*g/M, 0.f, 
       0.f, 0.f, 0.f, 1.f, 
       0.f, 0.f, (M + m)*g/(M*l), 0.f;

  x << 0.f, 0.f, .5f, 0.f;

  x_dot << 1.f, 0.f, 0.f, 0.f;

  B << 0.f, 1.f/M, 0.f, -1.f/(M*l);

  //K << -1.3003f,-2.3173f,-26.6124f,-5.6587f;
  //K << -874.92f,-307.24f,-843.75f,-176.62f;
  K << -12.237f,-20.598f,-110.906f,-22.799f;
  //K << -0.1225,-0.3952,-15.5213,-2.6976;

  setpoint << 0, 0, 0, 0;

  while(time < duration){
    
    myFile << time << "," << x(0) << "," << x(2) << "\n";

    u = -(K * (x - setpoint))(0);
    x_dot = (A * x) + (B * u);
    x += x_dot * dt;
    time += dt;
  }

  myFile.close();

  return 0;
}
