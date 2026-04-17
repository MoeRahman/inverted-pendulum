
#include "controller.h"
/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

void gain_settings(control_t control_mode, gain_t* gain){

  static const gain_t GAIN_TABLE[] = {
    [GENTLE]        = {-35.838, -27.848, -108.259, -24.2240},
    [AGGRESSIVE]    = {-224.90, -110.40, -322.490, -71.5010},
    [OPTIMAL]       = {-10.000, -23.712, -237.411, -113.135}
  };

  *gain = GAIN_TABLE[control_mode];
}
