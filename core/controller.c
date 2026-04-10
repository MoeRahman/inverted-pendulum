
#include "controller.h"
/*
- PID Controller
- Pole Placement + Full State Feedback
- Linear Quadratic Regulator (Optimal Pole-Placement Gains)
- Swing-up energy controls
*/

void gain_settings(control_t control_mode, gain_t* gain){

  static const gain_t GAIN_TABLE[] = {
    [GENTLE]       = {-1.3003, -2.3173, -26.6124, -5.65870},
    [AGRESSIVE]    = {-2.5390, -3.9352, -34.1295, -7.46760},
    [OPTIMAL]      = {-10.000, -23.712, -237.411, -113.135},
    [GENTLE_DC]    = {-0.1225, -1.3951, -15.5235, -2.69760},
    [AGRESSIVE_DC] = {-9.0346, -10.910, -55.2573, -12.2551},
    [OPTIMAL_DC]   = {-10.000, -10.267, -52.1470, -11.5630}
  };

  *gain = GAIN_TABLE[control_mode];
}
