#ifndef DC_MOTORS
#define DC_MOTORS

#include <Arduino.h>

#define PWM_FREQ        100     // PWM Frequency: 100Hz
#define PWM_RESOLUTION  8       // PWM Resolution: 8 bits (0-255)

class dc_motorClass{

private:
    int8_t dir = 0;
    uint8_t encoderA_Pin = -1,
            encoderB_Pin = -1, 
            IN1_Pin = -1, 
            IN2_Pin = -1, 
            PWM_Pin = -1,
            PWM_CHANNEL = -1;

    float speed;
    float target_speed;
public:
    void set_encoder_pins(uint8_t PinA, uint8_t PinB);
    void set_direction_pins(uint8_t PinA, uint8_t PinB);
    void set_pwm_pin(uint8_t Pin, uint8_t Channel);

    int8_t init();
    int8_t init(uint8_t EA, uint8_t EB, uint8_t IN1, uint8_t IN2);
    int8_t init(uint8_t EA, uint8_t EB, uint8_t IN1, uint8_t IN2, uint8_t PWM_Pin, uint8_t PWM_CHANNEL);
    

    void cal_speed();
    void cal_shaft_speed();
    void set_pwm();
    void run();
};


#endif