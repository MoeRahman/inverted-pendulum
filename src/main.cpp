#include "physics.hpp"
#include <iostream>
#include <fstream>

int main() {

  constexpr float g = 9.80665f;
  constexpr float m = .1f; //ball mass
  constexpr float M = 1.f; //cart mass
  constexpr float l = .5f; //pole length

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

  A << 0.f, 1.f, 0.f, 0.f, 
          0.f, 0.f, -m*g/M, 0.f, 
          0.f, 0.f, 0.f, 1.f, 
          0.f, 0.f, (M + m)*g/(M*l), 0.f;

  x << 0.f, 0.f, .5f, 0.f;

  x_dot << 0.f, 0.f, 0.f, 0.f;

  B << 0.f, 1.f/M, 0.f, -1.f/(M*l);

  K << -2.0f, -2.8f, -32.0f, -6.5f; 

  while(time < duration){
    
    myFile << time << "," << x(0) << "," << x(2) << "\n";

    u = -(K * x)(0);
    x_dot = (A * x) + (B * u);
    x += x_dot * dt;
    time += dt;
  }

  myFile.close();

  return 0;
}
