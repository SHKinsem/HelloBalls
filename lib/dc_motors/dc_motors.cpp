#include <dc_motors.h>

void dc_motorClass::set_encoder_pins(uint8_t PinA, uint8_t PinB){
    encoderA_Pin = PinA;
    encoderB_Pin = PinB;
}

void dc_motorClass::set_direction_pins(uint8_t PinA, uint8_t PinB){
    IN1_Pin = PinA;
    IN2_Pin = PinB;

}


void dc_motorClass::set_pwm_pin(uint8_t Pin, uint8_t Channel){
    PWM_Pin = Pin;
    PWM_CHANNEL = Channel;
}

int8_t dc_motorClass::init(){
    if(IN1_Pin != -1 && IN2_Pin != -1 && PWM_Pin != -1){
        pinMode(IN1_Pin,OUTPUT);
        pinMode(IN2_Pin,OUTPUT);
        pinMode(PWM_Pin,OUTPUT);
        return 1;
    }
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_Pin,PWM_CHANNEL);
    return -1;
}

int8_t dc_motorClass::init(uint8_t EA, uint8_t EB, uint8_t IN1, uint8_t IN2){
    set_encoder_pins(EA, EB);
    set_direction_pins(IN1, IN2);
    if(IN1_Pin != -1 && IN2_Pin != -1 && PWM_Pin != -1){
        pinMode(IN1_Pin,OUTPUT);
        pinMode(IN2_Pin,OUTPUT);
        pinMode(PWM_Pin,OUTPUT);
        return 1;
    }
    return -1;
}

int8_t dc_motorClass::init(uint8_t EA, uint8_t EB, uint8_t IN1, uint8_t IN2, uint8_t PWM_Pin, uint8_t PWM_CHANNEL){   // Overlaod function: handles the pin input
    set_encoder_pins(EA, EB);
    set_direction_pins(IN1, IN2);
    set_pwm_pin(PWM_Pin, PWM_CHANNEL);
    
    if(IN1_Pin != -1 && IN2_Pin != -1 && PWM_Pin != -1){
        pinMode(IN1_Pin,OUTPUT);
        pinMode(IN2_Pin,OUTPUT);
        pinMode(PWM_Pin,OUTPUT);
        return 1;
    }
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_Pin,PWM_CHANNEL);
    return -1;
}


