#include "physics.h"
#include <stdio.h>

int main(){

  printf("%lf,%lf,%lf,%lf\n", pendulum_params.g, 
    pendulum_params.m, pendulum_params.M, pendulum_params.L);

  printf("%lf,%lf,%lf,%lf\n", motor_params.k1, 
    motor_params.k2, motor_params.R, motor_params.r);

  return 0;
}