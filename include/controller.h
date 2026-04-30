#pragma once

typedef enum {
  K1, K2, K3, K4, K5, K6
}control_t;

const double *gain_settings(control_t control_mode);