#include "controller.h"

/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

double *set_controller_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    //Q = diag([1e2, 1e-2, 1e2, 1e3]) | R = 1e-3;
    {-295.60,   -433.93,  -3402.76,  -1091.80},

    //Q = diag([1e4, 1e-2, 1e2, 1e3]) | R = 7.0000e-04
    {-3.4812e+03,  -2.6381e+03,  -1.0089e+04,  -2.0201e+03},
    {-316.23,   -304.08,  -1718.52,  -371.03}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}

void DRDE(){
  
}