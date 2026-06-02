#include "controller.h"
#include "estimator.h"
#include "parameters.h"
#include "physics.h"

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

  fprintf(fpt, "time,x,vel,angle,input,setpoint,pos_err,x_est,vel_est,angle_est,x_meas,ang_vel,ang_vel_est,vel_err,theta_err,ang_vel_err\n");

  //Time elapsed variable and time step
  double time = 0;  //Units [sec]
  double dt = 0.0001; //Units [sec]

  //Write column titles
  vect4d_t state = {0,0,-1e-3,0};      //State {m, m/s, rad, rad/s}
  vect4d_t next_state = {0,0,0,0};     //Next State
  vect4d_t state_est = {0,0,0,0};      //State Estimate
  vect4d_t next_state_est = {0,0,0,0}; // d/dt (State Estimate)

  double y = 0.0; //Position Measurement

  double* Kc = set_controller_gain(K1); //Control Gain Vector
  double* Kf = set_estimator_gain(K1);  //Estimator Gain Vector

  double u = 0; //Input force 
  double err[4] = {0,0,0,0};

  //State Process Noise
  double noise = 0;
  //const double noise_variance[4] = {POS_NOISE, VEL_NOISE, ANGLE_NOISE, OMEGA_NOISE};

  //Measurement Noise
  double sensor_noise = 0;
  
  //Initial Setpoints for each state
  vect4d_t setpoint = {0,0,0,0};

  while(time < SIM_TIME){

    //gaussian_generator(mean, variance)
    noise = gaussian_generator(0, 1e-3);
    sensor_noise = gaussian_generator(0, 1e-2);

    //step
    if((time > 1) && (time <= 5)) setpoint.state.x = 1;
    if((time > 5) && (time <= 10)) setpoint.state.x = 0;

    //step-forward non-linear dynamics
    next_state = (vect4d_t){0,0,0,0};
    rk4_step(pendulum_dynamics, &state, &next_state, u, y, Kf, dt);
    state = next_state;

    //measure Position
    y = state.state.x + sensor_noise;

    //full-state estimation
    next_state_est = (vect4d_t){0,0,0,0};
    rk4_step(kalman_filter, &state_est, &next_state_est, u, y, Kf, dt);
    state_est = next_state_est;

    u = noise;
    for(size_t i = 0; i < 4; ++i){
      err[i] = state.arr[i] - state_est.arr[i];
      u += Kc[i]*(setpoint.arr[i] - state_est.arr[i]);
    }

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf,", time, state.state.x, state.state.x_dot, state.state.theta, u); 
    fprintf(fpt, "%lf,%lf,", setpoint.state.x, err[0]);
    fprintf(fpt, "%lf,%lf,%lf,%lf,", state_est.state.x, state_est.state.x_dot, state_est.state.theta, y);
    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf\n", state.state.theta_dot, state_est.state.theta_dot, err[1], err[2], err[3]);

    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}

