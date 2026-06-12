#include "controller.h"
#include "estimator.h"
#include "parameters.h"
#include "physics.h"

#define SIM_TIME 10
#define LOG_SIZE 16

typedef struct{
  double time;
  double position, velocity, angle, angular_velocity;
  double input, setpoint;
  double position_error, velocity_error, angle_error, angular_velocity_error;
  double position_estimate, velocity_estimate, angle_estimate, angular_velocity_estimate;
  double position_measurement; 
}data_packet_t;

//Log file initialize
data_packet_t log_file = {0};

int main(){

  //Generate seed for Guassian noise generation
  srand(time(NULL));

  //Create CSV File
  FILE *fpt = fopen("sim_file.bin", "wb");

  //Check if file opened successfully
  if (fpt == NULL) {
      fprintf(stderr, "Error opening file!\n");
      return 1;
  }

  //Time elapsed variable and time step
  double time = 0;  //Units [sec]
  const double dt = 0.0001; //Units [sec]

  //Write column titles
  vect4d_t state          = {0,0,-1e-3,0};      //State {m, m/s, rad, rad/s}
  vect4d_t next_state     = {0};     //Next State
  vect4d_t state_est      = {0};      //State Estimate
  vect4d_t next_state_est = {0}; // d/dt (State Estimate)

  double* Kc = set_controller_gain(K3); //Control Gain Vector
  double* Kf = set_estimator_gain(K1);  //Estimator Gain Vector

  double u = 0; //Input force 
  double err[4]  = {0};
  
  //Initial Setpoints for each state
  vect4d_t setpoint = {0};

  while(time < SIM_TIME){

    //gaussian_generator(mean, variance)
    double process_noise = gaussian_generator(PROCESS_NOISE_MEAN, PROCESS_NOISE_COVAR);
    double sensor_noise = gaussian_generator(POS_SENSOR_MEAN, POS_SENSOR_COVAR);

    //step
    //if((time > 1)) setpoint.state.x = 1;
    //if((time > 5) && (time <= 10)) setpoint.state.x = 0;
    //setpoint.state.x = 0.5*sin(2*M_PI*time/5);

    //measure cart position
    double y = state.state.x + sensor_noise;

    //step-forward non-linear dynamics
    next_state = (vect4d_t){0,0,0,0};
    rk4_step(pendulum_dynamics, &state, &next_state, u, y, Kf, dt);
    state = next_state;

    //full-state estimation
    next_state_est = (vect4d_t){0,0,0,0};
    rk4_step(kalman_filter, &state_est, &next_state_est, u, y, Kf, dt);
    state_est = next_state_est;

    u = process_noise;
    for(size_t i = 0; i < 4; ++i){
      err[i] = state.arr[i] - state_est.arr[i];
      u += Kc[i]*(setpoint.arr[i] - state_est.arr[i]);
    }

    //simulation time
    log_file.time = time;

    //state vector
    log_file.position         = state.state.x;
    log_file.velocity         = state.state.x_dot;
    log_file.angle            = state.state.theta;
    log_file.angular_velocity = state.state.theta_dot;

    //state error
    log_file.position_error         = err[0];
    log_file.velocity_error         = err[1];
    log_file.angle_error            = err[2]; 
    log_file.angular_velocity_error = err[3];

    //state estimates
    log_file.position_estimate         = state_est.state.x;
    log_file.velocity_estimate         = state_est.state.x_dot;
    log_file.angle_estimate            = state_est.state.theta;
    log_file.angular_velocity_estimate = state_est.state.theta_dot;

    //log control variables
    log_file.input = u;
    log_file.setpoint = setpoint.state.x;
    log_file.position_measurement = y;

    fwrite(&log_file, sizeof(double), LOG_SIZE, fpt);

    time += dt;
  }

  fclose(fpt);
  printf(".csv file created successfully.\n");

  return 0;
}

