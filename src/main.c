#include "controller.h"
#include "estimator.h"
#include "physics.h"
#include <math.h>
#include <stdio.h>

int main(){

  FILE *fpt = fopen("pendulum_sim.dat", "w");

  if (fpt == NULL) {
      fprintf(stderr, "Error opening file!\n");
      return 1;
  }

  const double duration = 15; // Units [sec]
  double time = 0;            // Units [sec]
  double dt = 0.01;           // Units [sec]

  fprintf(fpt, "Time,Pos_X,Angle,Voltage,Setpoint\n");

  state_t x = {0,0,0,0};
  double *K = NULL;
  gain_settings(GENTLE, &K);
  double u = 0;
      
  state_t setpoint = {0,0,0,0};

  while(time < duration){

    //step function
    if((time > 3) && (time < 6)){
      setpoint.pendulum.x = 1;
    }else if((time > 6) && (time < 9)){
      setpoint.pendulum.x = 0;
    }else if((time > 9) && (time < 12)){
      setpoint.pendulum.x = -1;
    }else if((time > 12)){
      setpoint.pendulum.x = 0;
    }

    for(size_t i = 0; i < 4; ++i){
      u -= K[i]*(x.arr[i] - setpoint.arr[i]);
    }

    state_t next_state = {0,0,0,0};
    rk4_step(&x, &next_state, pendulum_params, u, dt);

    fprintf(fpt, "%lf,%lf,%lf,%lf,%lf\n", time, x.pendulum.x, x.pendulum.theta, u, setpoint.pendulum.x);


    x = next_state;
    u = 0;
    time += dt;
  }

  fclose(fpt);
  printf(".dat file created successfully.\n");

  return 0;
}