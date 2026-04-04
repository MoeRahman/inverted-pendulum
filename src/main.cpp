#include "physics.hpp"
#include <cmath>
#include <iostream>
#include <fstream>

typedef enum {
  GENTLE, AGRESSIVE, OPTIMAL, 
  GENTLE_DC, AGRESSIVE_DC, OPTIMAL_DC
}control_t;

void gain_settings(control_t control_mode, Eigen::RowVector4d* Gains);

int main() {

  // Inverted Pendulum Params
  constexpr double g = 9.80665;
  constexpr double m = 0.1; //ball mass
  constexpr double M = 1; //cart mass
  constexpr double l = 0.5; //pole length

  // DC Motor Params
  constexpr double k1 = 1; //torque constant
  constexpr double k2 = 1; //back-emf constant
  constexpr double R = 1;  //motor internal resistance
  constexpr double r = 1; //torque to force ratio

  //write data to csv file
  std::ofstream myFile("data.csv");

  if (!myFile.is_open()) {
    std::cerr << "Error: Could not open the file!" << std::endl;
    return 1;
  }

  myFile << "Time,Pos_X,Angle,Voltage,Setpoint\n";

  //Simulate
  const double duration {30};
  double time{0};
  double dt{0.001};

  Eigen::Matrix4d A;
  Eigen::Vector4d x;
  Eigen::Vector4d x_dot;
  Eigen::Vector4d B;
  Eigen::RowVector4d K;
  double u{0}; // Input here is now the voltage to DC motor
  Eigen::Vector4d setpoint;

  // This is the linearized state
  A << 0, 1, 0, 0, 
       0, -k1*k2/(M*R*r*r), -m*g/M, 0, 
       0, 0, 0, 1, 
       0, k1*k2/(M*l*R*r*r), (M + m)*g/(M*l), 0;

  x << 0, 0, -0.2, 0;
  B << 0, k1/(r*M*R), 0, -k1/(r*M*l*R);

  // PICK A CONTROLLER TYPE: GENTLE, AGRESSIVE, OPTIMAL, GENTLE_DC, AGRESSIVE_DC, OPTIMAL_DC
  control_t ctrl_gain_type = OPTIMAL_DC;
  gain_settings(ctrl_gain_type, &K);

  setpoint << 1, 0, 0, 0;
  size_t count = 0;

  while(time < duration){

    //STEP INPUT
    if((count%3000) == 0){
      setpoint(0) *= -1;
    }

    //SINE INPUT
    //setpoint(0) = 0.5*sin(time);

    u = -(K * (x - setpoint))(0);
    u = std::clamp(u, -12.0, 12.0);

    myFile << time << ',' << x(0) << ',' << x(2) << ',' << u << ',' << setpoint(0) << '\n';

    Eigen::Vector4d x_next;
    rk4_step(&x, &x_next, u, M, m, l, g, dt);
    x = x_next;
    
    time += dt;
    count += 1;

  }

  myFile.close();

  return 0;
}

void gain_settings(control_t control_mode, Eigen::RowVector4d* Gains){

  switch(control_mode){
    case GENTLE:
      (*Gains) <<  -1.3003,-2.3173,-26.6124,-5.6587;
      break;
    case AGRESSIVE:
      (*Gains) <<  -2.5390,   -3.9352,  -34.1295,   -7.4676;
      break;
    case OPTIMAL:
      (*Gains) << -10.000,   -23.712,  -237.411,  -113.135;
      break;
    case GENTLE_DC:
      (*Gains) <<  -0.1225, -1.3951, -15.5235, -2.6976;
      break;
    case AGRESSIVE_DC:
      (*Gains) <<  -9.0346,  -10.9102,  -55.2573,  -12.2551;
      break;
    case OPTIMAL_DC:
      (*Gains) <<  -10.000,  -10.267,  -52.147,  -11.563;
      break;
  }

}