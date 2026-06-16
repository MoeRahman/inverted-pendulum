#include "parameters.h"


const pendulum_params_t pendulum_params = {
  .g = 9.80665, 
  .m = 10, 
  .M = 20, 
  .L = 0.5, 
  .b = 1, 
  .gamma = 1
};

const motor_params_t motor_params = {
  .k1 = 1, 
  .k2 = 1, 
  .R  = 1, 
  .r  = 1
};

double g     = pendulum_params.g;
double m     = pendulum_params.m;
double M     = pendulum_params.M;
double l     = pendulum_params.L;
double b     = pendulum_params.b;
double gamma = pendulum_params.gamma;

double mt = M + m;
double I  = (1.0 / 3.0) * m * l * l;

double den = 4.0 * I * mt - l * l * m * m + l * l * m * mt;

double a22 = (-4.0 * b * (I + (l * l * m) / 4.0)) / den;
double a23 = (-g * l * l * m * m) / den;
double a24 = (2.0 * gamma * l * m) / den;

double a42 = (2.0 * b * l * m) / den;
double a43 = (2.0 * g * l * m * mt) / den;
double a44 = (-4.0 * gamma * mt) / den;

double A = {
    {0.0, 1.0, 0.0, 0.0},
    {0.0, a22, a23, a24},
    {0.0, 0.0, 0.0, 1.0},
    {0.0, a42, a43, a44}
};

double B = {
    0.0,
    (4.0 * (I + (l * l * m) / 4.0)) / den,
    0.0,
    (-2.0 * l * m) / den
};



