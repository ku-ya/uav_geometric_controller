#ifndef GAINS_H
#define GAINS_H

// Position Proportional
#define kx_low 0.0
#define kx_mid 1.0
#define kx_high 6.5

// Position Derivative
#define kv_low 0.0
#define kv_mid 0.5
#define kv_high 4.8

// Attitude Proportional
#define kR_low 0.62
#define kR_mid 0.72
#define kR_high 1.95

// Attitude Derivative
#define kW_low 0.30
#define kW_mid 0.35
#define kW_high 0.75

void GetControllerGain(double *kx, double *kv, double *kiX, double *c1, double *kR, double *kW, double *kiR, double *c2)
{
    // Position:
    *kx = kx_high;//0.0;//7.4;//0.001;//1.0;
    *kv = kv_high;//0.0;//1.0;//0.0005;//0.5;
    *kiX = 0.1;
    *c1 = 0.0;
    // Attitude:
    *kR = kR_high;//1.25;
    *kW = kW_high;//0.75;
    *kiR = 0.1;//0.1;//0.2;
    *c2 = 0.0;//.1;
}



#endif // GAINS_H
