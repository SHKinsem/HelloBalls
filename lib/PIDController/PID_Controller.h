#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>

enum controllerType{DEFAULT_LOOP, SPEED_LOOP, POSITION_LOOP, TURQUE_LOOP};

class PIDController {
private:
    controllerType PIDcontrollerType = DEFAULT_LOOP;
    double kp; // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain
    double integralSum;
    double prevError;
public:
    void initController(controllerType type);
    void setControllerParams(double p, double i, double d);
    float maxOutput;
    float getOutput;
    // double compute(double currentValue, double setpoint);
    float compute(float currentValue, float setpoint);
};

#endif  // PID_CONTROLLER_H