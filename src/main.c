#include "controller.h"
#include "estimator.h"
#include "physics.h"
#include <stdlib.h>
#include <stdio.h>

int main(){

  //Generate seed for Guassian noise generation
  srand(time(NULL));

  FILE *fpt = fopen("pendulum_sim.csv", "w");

  if (fpt == NULL) {
      fprintf(stderr, "Error opening file!\n");
      return 1;
  }

  const double duration = 10; // Units [sec]
  double time = 0;            // Units [sec]
  double dt = 0.01;           // Units [sec]

  fprintf(fpt, "Time,Pos_X,Vel_X,Angle,Force,Setpoint\n");

  state_t x = {0,0,0,0};
  double *K = NULL;
  gain_settings(GENTLE, &K);
  double u = 0;
      
  state_t setpoint = {0,0,0,0};

  while(time < duration){

    state_t noise = {gaussian_generator(0, 0.004),   //State Process Noise
                     gaussian_generator(0, 0.005),  //State Process Noise
                     gaussian_generator(0, 0.002),   //State Process Noise
                     gaussian_generator(0, 0.005)};//State Process Noise

    setpoint.pendulum.x = sin(9*time/5);

    for(size_t i = 0; i < 4; ++i){
      x.arr[i] = x.arr[i] + noise.arr[i]; // Add process noise to current state
      u -= K[i]*(x.arr[i] - setpoint.arr[i]);
    }

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