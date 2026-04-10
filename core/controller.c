
#include "controller.h"
/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

void gain_settings(control_t control_mode, gain_t* gain){

  switch(control_mode){
    case GENTLE:
      gain->A = -1.3003;
      gain->B = -2.3173;
      gain->C = -26.6124;
      gain->D = -5.6587;
      break;
    case AGRESSIVE:
      gain->A = -2.5390;
      gain->B = -3.9352;
      gain->C = -34.1295;
      gain->D = -7.4676;
      break;
    case OPTIMAL:
      gain->A = -10.000;
      gain->B = -23.712;
      gain->C = -237.411;
      gain->D = -113.135;
      break;
    case GENTLE_DC:
      gain->A = -0.1225;
      gain->B = -1.3951;
      gain->C = -15.5235;
      gain->D = -2.6976;
      break;
    case AGRESSIVE_DC:
      gain->A = -9.0346;
      gain->B = -10.9102;
      gain->C = -55.2573;
      gain->D =-12.2551;
      break;
    case OPTIMAL_DC:
      gain->A = -10.000;
      gain->B = -10.267;
      gain->C = -52.147;
      gain->D = -11.563;
      break;
  }

}
