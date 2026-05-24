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

  //Time elapsed variable and time step
  double time = 0;  //Units [sec]
  double dt = 0.01; //Units [sec]

  //Write column titles
  fprintf(fpt, "i.time,ii.Pos_X,iii.vel_X,iv.angle,v.force,vi.setpoint,vii.error,viii.pos_est,ix.vel_est,x.angle_est\n");

  vect4d_t state = {0,0,-1e-3,0};      //State {m, m/s, rad, rad/s}
  vect4d_t next_state = {0,0,0,0};     //Next State

  vect4d_t state_est = {0,0,0,0};      //State Estimate
  vect4d_t dx_est = {0,0,0,0};         // d/dt (State Estimate)

  double y = 0.0;                      //Position Measurement

  double* Kc = set_controller_gain(K3); //Control Gain Vector
  double* Kf = set_estimator_gain(K1); //Estimator Gain Vector

  double u = 0;                         //Input force 

  //State Process Noise
  vect4d_t noise = {0,0,0,0};
  const double noise_variance[4] = {POS_NOISE, VEL_NOISE, ANGLE_NOISE, OMEGA_NOISE};

  //Measurement Noise
  double sensor_noise = 0;
  
  //Initial Setpoints for each state
  vect4d_t setpoint = {0,0,0,0};

  while(time < SIM_TIME){

    for(size_t i = 0; i < 4; ++i){
      noise.arr[i] = gaussian_generator(0, noise_variance[i]);
    }

    sensor_noise = gaussian_generator(0, POS_SENSOR_NOISE);

    if((time >= 2) && (time < 4))setpoint.state.x = 1;

    //Measure Position
    y = state.state.x + sensor_noise;

    //Full-State Estimation
    rk4_step(kalman_filter, &state_est, &state_est, u, y, Kf, dt);

    u = 0;
    for(size_t i = 0; i < 4; ++i){
      u -= Kc[i]*(state.arr[i] - setpoint.arr[i]);
    }

    rk4_step(pendulum_dynamics, &state, &next_state, u, y, &Kf, dt);

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf,", time, state.state.x, state.state.x_dot, state.state.theta, u/1000); 
    fprintf(fpt, "%lf,%lf,", setpoint.state.x, setpoint.state.x - state.state.x);
    fprintf(fpt, "%lf,%lf,%lf,%lf\n", state_est.state.x, state_est.state.x_dot, state_est.state.theta, y);

    for(size_t i = 0; i < 4; ++i){
      state.arr[i] = next_state.arr[i] + noise.arr[i];
    }

    next_state = (vect4d_t){0,0,0,0};
    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}