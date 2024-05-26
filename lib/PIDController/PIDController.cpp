#include "PIDController.h"

void PIDController::initController(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
    integralSum = 0;
    prevError = 0;
}

int16_t PIDController::compute(float currentValue, float setpoint) {
    float error = setpoint - currentValue;
    // Proportional term
    float pTerm = kp * error;

    // Integral term
    integralSum += error;

    // Anti-windup 
    if(integralSum > maxOutput/ki) {
        integralSum = maxOutput/ki;
    } else if(integralSum < -maxOutput/ki) {
        integralSum = -maxOutput/ki;
    }

    float iTerm = ki * integralSum;


    // Derivative term
    float dTerm = kd * (error - prevError);
    prevError = error;

    // Calculate total control output
    int16_t output = pTerm + iTerm + dTerm;

    if(output > maxOutput) {
        output = maxOutput;
    } else if(output < -maxOutput) {
        output = -maxOutput;
    }
    this->getOutput = error;   // For debugging purpose



    return output;
}

