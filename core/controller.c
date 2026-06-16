#include "controller.h"

/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

double *set_controller_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    {-315.57,   -265.41,  -1364.91,   -212.34},
    {-7040.2,  -3473.0,  -8689.4,  -1419.8},
    {-316.23,   -304.08,  -1718.52,   -371.03}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}