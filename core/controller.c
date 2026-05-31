#include "controller.h"

/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

double *set_controller_gain(gain_t gain){

  static double GAIN_TABLE[3][4] = {
    {-100.000,  -115.775,  -940.070,  -140.533},
    {-1000,-735.01,-2879.72,-598.14},
    {-3162.3,-1914.9,-5952.5,-1251.1}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}