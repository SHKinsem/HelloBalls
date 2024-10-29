#ifndef DC_MOTORS
#define DC_MOTORS

#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include <PID_Controller.h>

#define PWM_FREQ        100     // PWM Frequency: 100Hz
#define PWM_RESOLUTION  8       // PWM Resolution: 8 bits (0-255)

class dc_motorClass{

private:
    int8_t dir = 0;
    float speed;            // Angular speed
    float target_speed;     // Target angular speed
    
    dc_motorClass* ptrToSelf = nullptr;
public:
    PIDController speedController;
    uint8_t encoderA_Pin = -1,
            encoderB_Pin = -1, 
            IN1_Pin = -1, 
            IN2_Pin = -1, 
            PWM_Pin = -1,
            PWM_CHANNEL = -1;
    volatile long encoderCount = 0,
                  prevEncoderCount = 0,
                  currTime = 0,
                  prevTime = 0;
    volatile float degree = 0,
                   prevDegree = 0;
    void set_encoder_pins(uint8_t pinA, uint8_t pinB);
    void set_direction_pins(uint8_t pinA, uint8_t pinB);
    void set_pwm_pin(uint8_t pin, uint8_t channel);
    float get_speed();
    float get_target_speed();

    int8_t init(int ea_pin, int eb_pin, int in1_pin, int in2_pin, int pwm_pin, int channel);
    void IRAM_ATTR encoderInterrupt(void);
    void init_speed_controller();
    void set_pid(float parms[6]);
    void cal_speed();
    void set_speed(int data);
    void cal_shaft_speed();
    int set_pwm();
    void run();
};


#endif