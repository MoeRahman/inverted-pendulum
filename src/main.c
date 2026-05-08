#include "controller.h"
#include "estimator.h"
#include "physics.h"
#include <stdio.h>
#include <stdlib.h>

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
  double time = 0;  //Units [sec]
  double dt = 0.01; //Units [sec]

  //Write column titles
  fprintf(fpt, "Time,Pos_X,Vel_X,Angle,Force,Setpoint,ERROR\n");

  vect4d_t x = {0,0,-1e-3,0};          //State Vector {m, m/s, rad, rad/s}
  vect4d_t x_est = {0,0,0,0};          //State Estimation Vector
  vect4d_t next_state = {0,0,0,0};     //Next State Vector
  vect4d_t y = {0,0,0,0};              //Measurement Vector
  const double *K = gain_settings(K3); //Gain Vector
  double u = 0;                        //Input force 

  //State Process Noise
  vect4d_t noise = {0,0,0,0};
  const double noise_variance[4] = {POS_NOISE, VEL_NOISE, ANGLE_NOISE, OMEGA_NOISE};

  //Measurement Noise
  vect4d_t measurement_noise = {0,0,0,0};
  const double sensor_variance[4] = {POS_SENSOR_NOISE, 0, ANGLE_SENSOR_NOISE, 0};
  
  //Initial Setpoints for each state
  vect4d_t setpoint = {0,0,0,0};

  while(time < SIM_TIME){

    for(size_t i = 0; i < 4; ++i){
      noise.arr[i] = gaussian_generator(0, noise_variance[i]);
      measurement_noise.arr[i] = gaussian_generator(0, sensor_variance[i]);
    }

    if((time >= 2) && (time < 4))setpoint.state.x = 1;

    for(size_t i = 0; i < 4; ++i){
      //Full-State observation assumed
      y.arr[i] = x.arr[i] + measurement_noise.arr[i];
      
      x_est.arr[i] = y.arr[i];

      u -= K[i]*(x_est.arr[i] - setpoint.arr[i]);
    }

    rk4_step(&x, &next_state, pendulum_params, u, dt, ENABLE_DAMPING);

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", time, 
      x.state.x, x.state.x_dot, x.state.theta, u, 
      setpoint.state.x, setpoint.state.x - x.state.x);

    x = next_state;
    next_state = (vect4d_t){0,0,0,0};
    u = 0;
    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}