#include "physics.hpp"
#include <cmath>
#include <iostream>
#include <fstream>

typedef enum {GENTLE, AGRESSIVE, OPTIMAL}control_t;

void gain_settings(control_t control_mode, Eigen::RowVector4d* Gains);

int main() {

  constexpr double g = 9.80665;
  constexpr double m = 0.1; //ball mass
  constexpr double M = 1; //cart mass
  constexpr double l = 0.5; //pole length

  //write data to csv file
  std::ofstream myFile("data.csv");

  if (!myFile.is_open()) {
    std::cerr << "Error: Could not open the file!" << std::endl;
    return 1;
  }

  myFile << "Time,Pos_X,Angle\n";

  //Simulate
  const double duration {30};
  double time{0};
  double dt{0.001};

  Eigen::Matrix4d A;
  Eigen::Vector4d x;
  Eigen::Vector4d x_dot;
  Eigen::Vector4d B;
  Eigen::RowVector4d K;
  double u{0};
  Eigen::Vector4d setpoint;

  // This is the linearized state
  A << 0, 1, 0, 0, 
       0, 0, -m*g/M, 0, 
       0, 0, 0, 1, 
       0, 0, (M + m)*g/(M*l), 0;

  x << 0, 0, -0.2, 0;
  B << 0, 1/M, 0, -1/(M*l);

  control_t ctrl_gain_type = OPTIMAL;
  gain_settings(ctrl_gain_type, &K);

  setpoint << 1, 0, 0, 0;
  size_t count = 0;

  while(time < duration){

    if((count%1000) == 0){
      setpoint(0) *= -1;
    }

    u = -(K * (x - setpoint))(0);
    u = std::clamp(u, -20.0, 20.0);

    myFile << time << "," << x(0) << "," << x(2) << "\n";

    rk4_step(&x, &x, u, M, m, l, g, dt);

    time += dt;
    count += 1;

  }

  myFile.close();

  return 0;
}

void gain_settings(control_t control_mode, Eigen::RowVector4d* Gains){

  switch(control_mode){
    case GENTLE:
      //GENTLE CONTROLS
      (*Gains) <<  -1.3003,-2.3173,-26.6124,-5.6587;
      //(*Gains) << -12.237f,-20.598f,-110.906f,-22.799f;
      //(*Gains) << -0.1225,-0.3952,-15.5213,-2.6976;
      break;
    case AGRESSIVE:
      //AGRESSIVE CONTROLS
      (*Gains) <<  -2.5390,   -3.9352,  -34.1295,   -7.4676;
      //(*Gains) << -9.3780,  -11.5089,  -64.0490,  -14.2545;
      //(*Gains) << -30.166,  -20.390,  -100.643,  -10.695;
      break;
    case OPTIMAL:
      (*Gains) << -10.000,   -23.712,  -237.411,  -113.135;
      break;
  }

}