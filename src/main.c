#include "controller.h"
#include "estimator.h"
#include "physics.h"
#include <stdlib.h>
#include <stdio.h>


#define MAX_FORCE 100
#define MIN_FORCE -100

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
  double time = 0;            // Units [sec]
  double dt = 0.01;           // Units [sec]

  //Write column titles
  fprintf(fpt, "Time,Pos_X,Vel_X,Angle,Force,Setpoint\n");

  //Declare and Initialize state variables, Gain vector, and input force
  state_t x = {0,0,0,0};
  double *K = NULL;
  gain_settings(GENTLE, &K);
  double u = 0;
  
  //Initial Setpoints for each state
  state_t setpoint = {0,0,0,0};

  while(time < 10){

    state_t noise = {gaussian_generator(0, 0.01),   //State Process Noise
                     gaussian_generator(0, 0.005),   //State Process Noise
                     gaussian_generator(0, 0.02),   //State Process Noise
                     gaussian_generator(0, 0.001)};  //State Process Noise

    //x-position setpoint follows sinewave
    //setpoint.pendulum.x = 0.5*sin((k)*time);
    if(time > 5) setpoint.pendulum.x = 1;

    for(size_t i = 0; i < 4; ++i){
      x.arr[i] = x.arr[i] + noise.arr[i]; // Add process noise to current state
      u -= K[i]*(x.arr[i] - setpoint.arr[i]);
    }

    u = ((u>MAX_FORCE) ? MAX_FORCE : (u<MIN_FORCE) ? MIN_FORCE : u);

    state_t next_state = {0,0,0,0};
    rk4_step(&x, &next_state, pendulum_params, u, dt);

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf,%lf\n", time, 
      x.pendulum.x, x.pendulum.x_dot, x.pendulum.theta, u, setpoint.pendulum.x);


    x = next_state;
    u = 0;
    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}