#include "controller.h"
/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

void gain_settings(control_t control_mode, double **gain){

  static const double GAIN_TABLE[3][4] = {
    {-35.838, -27.848, -108.259, -24.2240},
    {-224.90, -110.40, -322.490, -71.5010},
    {-10.000, -23.712, -237.411, -113.135}
  };

  if((control_mode >= 0) && (control_mode < 3)){
    *gain = GAIN_TABLE[control_mode];
  }
}
