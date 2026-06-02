#include "controller.h"

/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

double *set_controller_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    {-707.11,   -570.40,  -2542.44,   -559.35},
    {-223.61,   -238.73,  -1533.44,   -325.13},
    {-316.23,   -304.08,  -1718.52,   -371.03}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}