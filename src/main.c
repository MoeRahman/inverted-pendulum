#include "physics.h"

int main(){

  pendulum_params_t pendulum_1{
    .g = 9.80665f,
    .m = 0.1,
    .M = 1,
    .l = 0.5f 
  };

  motor_params_t motor_1{
    .k1 = 1f,
    .k2 = 1f,
    .R  = 1f,
    .r  = 1f
  };

  


  
  return 0;
}