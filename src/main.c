#include "physics.h"
#include "controller.h"
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

  fprintf(fpt, "Time,Pos_X,Angle,Voltage,Setpoint\n");

  pendulum_state_t x = {0,0,0.05,0};
  gain_t K = {0,0,0,0};
  gain_settings(OPTIMAL_DC, &K);
  double u = 0;
      
  pendulum_state_t setpoint = {1,0,0,0};

  while(time < duration){

    if((count%2500) == 0){
      setpoint.x *= -1;
    }

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf\n", time, x.x, x.theta, u, setpoint.x);

    u = -1*(K.A*(x.x - setpoint.x) + K.B*(x.x_dot - setpoint.x_dot) + 
            K.C*(x.theta - setpoint.theta) + K.D*(x.theta_dot - setpoint.theta_dot));

    pendulum_state_t next_state = {0,0,0,0};
    rk4_step(&x, &next_state, pendulum_params, u, dt);
    x = next_state;

    time += dt;
    count += 1;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}