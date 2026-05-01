#include "controller.h"
#include "estimator.h"
#include "physics.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define POS_NOISE   1e-4
#define VEL_NOISE   1e-3
#define ANGLE_NOISE 1e-5
#define OMEGA_NOISE 1e-4

#define ENABLE_DAMPING true
#define DISABLE_DAMPING false

#define SIM_TIME 10

int main(){

  //Generate seed for Guassian noise generation
  srand(time(NULL));

  //Create CSV File
  FILE *fpt = fopen("pendulum_sim.csv", "w");

  //Check if file opened successfully
  if (fpt == NULL) {
      fprintf(stderr, "Error opening file!\n");
      return 1;
  }

  //Time elapsed variable and time step
  double time = 0;            //Units [sec]
  double dt = 0.01;           //Units [sec]

  //Write column titles
  fprintf(fpt, "Time,Pos_X,Vel_X,Angle,Force,Setpoint,ERROR\n");

  state_t x = {0,0,-1e-3,0};                //State Vector {x_pos, x_vel, theta, angular_velocity}
  state_t x_est = {0,0,0,0};                //State Estimation Vector
  state_t next_state = {0,0,0,0};           //Next State Vector
  state_t y = {0,0,0,0};                    //Measurement Vector
  const double *K = gain_settings(K3);      //Gain Vector
  double u = 0;                             //Input force 

  //State Process Noise
  state_t noise = {0,0,0,0};
  double noise_std_dev[4] = {POS_NOISE, VEL_NOISE, ANGLE_NOISE, OMEGA_NOISE};
  
  //Initial Setpoints for each state
  state_t setpoint = {0,0,0,0};

  while(time < SIM_TIME){

    for(size_t i = 0; i < 4; ++i){
      noise.arr[i] = gaussian_generator(0, noise_std_dev[i]);
    }       

    if((time >= 2) && (time < 4)){setpoint.pendulum.x = -1;}
    if((time >= 4) && (time < 6)){setpoint.pendulum.x =  0.0;}
    if((time >= 6) && (time < 8)){setpoint.pendulum.x =  1;}
    if((time >= 8) && (time < 10)){setpoint.pendulum.x = 0.0;}
    //setpoint.pendulum.x += dt;

    for(size_t i = 0; i < 4; ++i){
      //Add process noise to current state
      x.arr[i] = x.arr[i] + noise.arr[i];
      u -= K[i]*(x.arr[i] - setpoint.arr[i]);
    }

    rk4_step(&x, &next_state, pendulum_params, u, dt, ENABLE_DAMPING);

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", time, 
      x.pendulum.x, x.pendulum.x_dot, x.pendulum.theta, u, 
      setpoint.pendulum.x, setpoint.pendulum.x - x.pendulum.x);

    x = next_state;
    next_state = (state_t){0,0,0,0};
    u = 0;
    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}