#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>
class PIDController {
private:
    double kp; // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain
    double integralSum;
    double prevError;
public:
    int16_t maxOutput;
    int getOutput;
    void initController(double p, double i, double d);
    // double compute(double currentValue, double setpoint);
    int16_t compute(float currentValue, float setpoint);
};

#endif  // PID_CONTROLLER_H