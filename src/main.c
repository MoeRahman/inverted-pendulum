#include "physics.h"
#include <stdio.h>

int main(){

  FILE *fpt = fopen("pendulum_sim.dat", "w");

  if (fpt == NULL) {
      printf("Error opening file!\n");
      return 1;
  }

  const double duration = 30;
  double time = 0;
  double dt = 0.001;
  size_t count = 0;

  typedef struct{
    double K1;
    double K2;
    double K3;
    double K4;
  }gain_t;

  fprintf(fpt, "Time,Pos_X,Angle,Voltage,Setpoint\n");

  pendulum_state_t x = {0,0,0.2,0};

  double u = 0;

  gain_t gain = {
    .K1 = -1.3003,
    .K2 = -2.3173,
    .K3 = -26.6124,
    .K4 = -5.6587,
  };

  pendulum_state_t setpoint = {1,0,0,0};

  while(time < duration){

    if((count%5000) == 0){
      setpoint.x *= -1;
    }

    pendulum_state_t next_state = {0,0,0,0};

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf\n", time, x.x, x.theta, u, setpoint.x);
    fflush(fpt);

    u = -1*(gain.K1*(x.x - setpoint.x) + 
            gain.K2*(x.x_dot - setpoint.x_dot) + 
            gain.K3*(x.theta - setpoint.theta) + 
            gain.K4*(x.theta_dot - setpoint.theta_dot));

    rk4_step(&x, &next_state, pendulum_params, u, dt);
    x = next_state;

    time += dt;
    count += 1;
  }

  fclose(fpt);
  printf("CSV file created successfully.\n");

  return 0;
}