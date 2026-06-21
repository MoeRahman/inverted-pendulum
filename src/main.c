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

void update_log(data_packet_t* file, double time, vect4d_t state, double* err, 
  vect4d_t state_estimate, double input, double setpoint, double measurement);

int main(void){

  //Generate seed for Guassian noise generation
  srand(time(NULL));

  //Create Binary File
  FILE *fpt = fopen("sim_file.bin", "wb");

  //Check if file opened successfully
  if (fpt == NULL) {
      fprintf(stderr, "Error opening file!\n");
      return 1;
  }

  //Time elapsed variable and time step
  double time                = 0;      //Units [sec]
  const double dt            = 0.0001; //Units [sec] 10kHz
  const double controller_dt = 0.0001;   //Units [sec] 1KHz

  const size_t time_steps = (size_t)(SIM_TIME/dt);
  const size_t zoh_steps  = (size_t)(controller_dt/dt);

  //Write column titles
  vect4d_t state          = {0,0,-1e-3,0};    //State {m, m/s, rad, rad/s}
  vect4d_t next_state     = {0};              //Next State
  vect4d_t state_est      = {0};              //State Estimate
  vect4d_t next_state_est = {0};              // d/dt (State Estimate)

  double* Kc = set_controller_gain(K2); //Control Gain Vector
  double* Kf = set_estimator_gain(K2);  //Estimator Gain Vector

  double u = 0; //Input force 

  //Estimation Error
  double err[4]  = {0};
  double rmse[4] = {0};
  
  //Initial Setpoints for each state
  vect4d_t setpoint = {0};

  while(time < SIM_TIME){

    //gaussian_generator(mean, variance)
    double process_noise = gaussian_generator(PROCESS_NOISE_MEAN, PROCESS_NOISE_COVAR);
    double sensor_noise = gaussian_generator(POS_SENSOR_MEAN, POS_SENSOR_COVAR);

    //step
    if(time > 1) setpoint.state.x = 0.5;
    if((time >= 5) & (time < 10)) setpoint.state.x = 0;

    //measure cart position
    double y = state.state.x + sensor_noise;

    u = process_noise;
    for(size_t i = 0; i < 4; ++i){
      //Estimation Error
      err[i] = state.arr[i] - state_est.arr[i];
      rmse[i] += pow(err[i], 2.0);
      // u = -Kx
      u += Kc[i]*(setpoint.arr[i] - state_est.arr[i]);
    }

    //step-forward non-linear dynamics
    next_state = (vect4d_t){0,0,0,0};

    rk4_step(pendulum_dynamics, &state, &next_state, u, y, Kf, dt);
    state = next_state;

    //full-state estimation
    next_state_est = (vect4d_t){0,0,0,0};
    rk4_step(kalman_filter, &state_est, &next_state_est, u, y, Kf, dt);
    state_est = next_state_est;
    
    update_log(&log_file, time, state, err, state_est, u, setpoint.arr[0], y);
    fwrite(&log_file, sizeof(double), LOG_SIZE, fpt);

    time += dt;
  }

  const char* lables[] = {"pos\t","vel\t","angle\t","omega\t"};
  printf("\n______ERROR_____\n");

  for(size_t i = 0; i < 4; ++i){
    rmse[i] = sqrt(rmse[i])/time_steps;
    printf("%s%.6lf\n",lables[i], rmse[i]);
  }

  fclose(fpt);
  printf("\n==SIM COMPLETE==\n");

  return 0;
}

void update_log(data_packet_t* file, double time, vect4d_t state, double* err, 
 vect4d_t state_estimate, double input, double setpoint, double measurement){

    //simulation time
    file->time = time;

    //state vector
    file->position         = state.state.x;
    file->velocity         = state.state.x_dot;
    file->angle            = state.state.theta;
    file->angular_velocity = state.state.theta_dot;

    //state error
    file->position_error         = err[0];
    file->velocity_error         = err[1];
    file->angle_error            = err[2]; 
    file->angular_velocity_error = err[3];

    //state estimates
    file->position_estimate         = state_estimate.state.x;
    file->velocity_estimate         = state_estimate.state.x_dot;
    file->angle_estimate            = state_estimate.state.theta;
    file->angular_velocity_estimate = state_estimate.state.theta_dot;

    //log control variables
    file->input                = input;
    file->setpoint             = setpoint;
    file->position_measurement = measurement;

  }

