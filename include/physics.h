#pragma once


typedef struct{
  const double g; //gravity     [m/s^2]
  const double m; //ball mass   [kg]
  const double M; //cart mass   [kg]
  const double L; //pole length [m]
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
    double theta;     //pole angle       [θ]
    double theta_dot; //angular velocity [ω]
  }pendulum;

}state_t;


extern const pendulum_params_t pendulum_params;
extern const motor_params_t motor_params;


void pendulum_dynamics(state_t const *curr_state, 
                       state_t  *next_state, 
                       const pendulum_params_t pendulum_params, 
                       const double Force);


void rk4_step(state_t const *curr_state, 
              state_t* next_state,
              const pendulum_params_t pendulum_parms, 
              const double F, const double dt);