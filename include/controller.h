#pragma once

typedef enum {
  GENTLE, AGRESSIVE, OPTIMAL, 
  GENTLE_DC, AGRESSIVE_DC, OPTIMAL_DC
}control_t;

typedef struct{
  double A;
  double B;
  double C;
  double D;
}gain_t;

void gain_settings(control_t control_mode, gain_t* gain);