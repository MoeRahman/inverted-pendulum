#include "controller.h"
#include "physics.h"
#include <math.h>
#include <stdio.h>

int main(){

  FILE *fpt = fopen("pendulum_sim.dat", "w");

  if (fpt == NULL) {
      fprintf(stderr, "Error opening file!\n");
      return 1;
  }

  const double duration = 15;
  double time = 0;
  double dt = 0.001;

  fprintf(fpt, "Time,Pos_X,Angle,Voltage,Setpoint\n");

  pendulum_state_t x = {0,0,0,0};
  gain_t K = {0,0,0,0};
  gain_settings(GENTLE, &K);
  double u = 0;
      
  pendulum_state_t setpoint = {0,0,0,0};

  while(time < duration){

    //step function
    if((time > 3) && (time < 6)){
      setpoint.x = 1;
    }else if((time > 6) && (time < 9)){
      setpoint.x = 0;
    }else if((time > 9) && (time < 12)){
      setpoint.x = -1;
    }else if((time > 12)){
      setpoint.x = 0;
    }

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf\n", time, x.x, x.theta, u, setpoint.x);

    u = -1*(K.A*(x.x - setpoint.x) + K.B*(x.x_dot - setpoint.x_dot) + 
            K.C*(x.theta - setpoint.theta) + K.D*(x.theta_dot - setpoint.theta_dot));

    pendulum_state_t next_state = {0,0,0,0};
    rk4_step(&x, &next_state, pendulum_params, u, dt);
    x = next_state;

    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}