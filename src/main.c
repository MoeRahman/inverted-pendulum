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
  fprintf(fpt, "Time,Pos_X,Vel_X,Angle,Force,Setpoint,ERROR\n");

  vect4d_t x = {0,0,-1e-3,0};          //State {m, m/s, rad, rad/s}
  vect4d_t x_est = {0,0,0,0};          //State Estimate
  vect4d_t dx_est = {0,0,0,0};         // d/dt (State Estimate)
  vect4d_t next_state = {0,0,0,0};     //Next State
  double y = 0.0;                      //Position Measurement

  double *Kc = set_controller_gain(K3); //Control Gain Vector
  vect4d_t Kf = set_estimator_gain(K1); //Estimator Gain Vector
  double u = 0;                         //Input force 

  //State Process Noise
  vect4d_t noise = {0,0,0,0};
  const double noise_variance[4] = {POS_NOISE, VEL_NOISE, 
                                    ANGLE_NOISE, OMEGA_NOISE};

  //Measurement Noise
  double sensor_noise = 0;
  
  //Initial Setpoints for each state
  vect4d_t setpoint = {0,0,0,0};

  while(time < SIM_TIME){

    for(size_t i = 0; i < 4; ++i){
      noise.arr[i] = gaussian_generator(0, noise_variance[i]);
    }
    sensor_noise = gaussian_generator(0, POS_SENSOR_NOISE);

    if((time >= 2) && (time < 4))setpoint.state.x = 0.1;

    //Measure Position
    y = x.state.x + sensor_noise;

    //d/dt State Estimation
    dx_est = kalman_filter(&x_est, &Kf, u, y);

    for(size_t i = 0; i < 4; ++i){
      u -= Kc[i]*(x.arr[i] - setpoint.arr[i]);
    }

    rk4_step(&x, &next_state, u, dt);

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", 
            time, x.state.x, x.state.x_dot, x.state.theta, u, 
            setpoint.state.x, setpoint.state.x - x.state.x,
            x_est.state.x, x_est.state.x_dot, x_est.state.theta);

    for(size_t i = 0; i < 4; ++i){
      x.arr[i] = next_state.arr[i] + noise.arr[i];
      next_state.arr[i] = 0;
    }

    u = 0;
    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}