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
    float maxOutput;
    float getOutput;
    void initController(double p, double i, double d);
    // double compute(double currentValue, double setpoint);
    float compute(float currentValue, float setpoint);
};

#endif  // PID_CONTROLLER_H