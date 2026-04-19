#pragma once

typedef enum {
  GENTLE, AGGRESSIVE, OPTIMAL, 
  GENTLE_DC, AGGRESSIVE_DC, OPTIMAL_DC
}control_t;

void gain_settings(control_t control_mode, double **gain);