#pragma once

typedef struct{
  const double G; //gravity     [m/s^2]
  const double m; //ball mass   [kg]
  const double M; //cart mass   [kg]
  const double L; //pole length [m]
  const double b; //linear viscous damping      [N*s/m]
  const double g; //rotational viscous damping  [N*m*s/rad]
}pendulum_params_t;


typedef struct{
  const double k1; //torque constant           [N*m/A]
  const double k2; //back-emf constant         [V/kRPM]
  const double R;  //motor internal resistance [Ω]
  const double r;  //torque to force ratio
}motor_params_t;


typedef union{
  double arr[4];

  struct{
    double x;         //position         [m]
    double x_dot;     //velocity         [m/s]
    double theta;     //pole angle       [rad]
    double theta_dot; //angular velocity [rad/s]
  }state;

}vect4d_t;