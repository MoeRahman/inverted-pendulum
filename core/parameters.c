#include "parameters.h"


const pendulum_params_t pendulum_params = {
  .g = 9.80665, 
  .m = 10, 
  .M = 20, 
  .L = 0.5, 
  .b = 2, 
  .gamma = 1
};


const motor_params_t motor_params = {
  .k1 = 1, 
  .k2 = 1, 
  .R  = 1, 
  .r  = 1
};


