#include "controller.h"

/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

const double *set_controller_gain(gain_t gain){

  static const double GAIN_TABLE[3][4] = {
    {-7.0711e+03,  -3.9489e+03,  -1.1089e+04,  -2.3468e+03},
    {-1000,   -735.01,  -2879.72,   -598.14},
    {-3162.3,  -1914.9,  -5952.5,  -1251.1}
  };

  if((gain >= 0) && (gain < 3)){
    return GAIN_TABLE[gain];
  }else{
    return NULL;
  }
}