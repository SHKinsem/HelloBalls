#include <dc_motors.h>
#include "FunctionalInterrupt.h"


dc_motorClass* dc_motorClass_ptr = nullptr;

static void interuptHandler(void){
    dc_motorClass_ptr->encoderInterrupt();
}

void dc_motorClass::set_encoder_pins(uint8_t pinA, uint8_t pinB){
    encoderA_Pin = pinA;
    encoderB_Pin = pinB;
}

void dc_motorClass::set_direction_pins(uint8_t pinA, uint8_t pinB){
    IN1_Pin = pinA;
    IN2_Pin = pinB;
}

void dc_motorClass::set_pwm_pin(uint8_t pin, uint8_t channel){
    PWM_Pin = pin;
    PWM_CHANNEL = channel;
}

float dc_motorClass::get_speed(){
    return speed;
}

float dc_motorClass::get_target_speed(){
    return target_speed;
}

int8_t dc_motorClass::init(int ea_pin, int eb_pin, int in1_pin, int in2_pin, int pwm_pin, int channel){   // Overlaod function: handles the pin input
    set_encoder_pins(ea_pin, eb_pin);
    set_direction_pins(in1_pin, in2_pin);
    set_pwm_pin(pwm_pin, channel);
    uint8_t state = 0;

    if(IN1_Pin != -1 && IN2_Pin != -1){
        pinMode(IN1_Pin,OUTPUT);
        pinMode(IN2_Pin,OUTPUT);
        pinMode(PWM_Pin,OUTPUT);
        state++;
    }

    if(state && PWM_Pin != -1){
        ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(PWM_Pin,PWM_CHANNEL);
        state++;
    }
    if(state && encoderA_Pin != -1 && encoderB_Pin != -1){
        pinMode(encoderA_Pin, INPUT_PULLUP);
        pinMode(encoderB_Pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(encoderA_Pin), std::bind(&dc_motorClass::encoderInterrupt, this), CHANGE);
        init_speed_controller();
    }
    return state;
}

void IRAM_ATTR dc_motorClass::encoderInterrupt(){
    if (digitalRead(encoderA_Pin) == digitalRead(encoderB_Pin)) {
        encoderCount++;
    } 
    else encoderCount--;
}

void dc_motorClass::init_speed_controller(){
    speedController.initController(DEFAULT_LOOP);
    speedController.maxOutput = 200; // Max PWM value
}

void dc_motorClass::set_pid(float parms[6]){
    this->speedController.setControllerParams(parms[0], parms[1], parms[2]);
}

void dc_motorClass::cal_speed(){
    currTime = millis();
    long passedTime = currTime - prevTime;
    degree = (prevDegree + (encoderCount*(360.0/782.0)))/2;   // Average the degree value

    speed = (degree - prevDegree)*100.0/passedTime;
    prevDegree = degree;

    // speed = (encoderCount - prevEncoderCount)*1000/passedTime;
    // prevEncoderCount = encoderCount;

    prevTime = currTime;
}

void dc_motorClass::set_speed(int data){
    target_speed = data;
}

int dc_motorClass::set_pwm(){
    float PWM_SPEED = this->speedController.compute(speed, target_speed);
    if(target_speed == 0){
        PWM_SPEED = 0;
    }
    ledcWrite(PWM_CHANNEL, abs(PWM_SPEED));
    if(PWM_SPEED < 0) return 0;
    else if (PWM_SPEED == 0) return -1;
    else return 1;
}

void dc_motorClass::run(){
    cal_speed();
    int dir = set_pwm();
    if(dir != -1){
        digitalWrite(IN1_Pin, !dir);
        digitalWrite(IN2_Pin, dir);
    }
    else{
        digitalWrite(IN1_Pin,LOW);
        digitalWrite(IN2_Pin,LOW);
    }

    // if(abs(speed) > 180 && speedClampCounter > 0){
    //     speedClampCounter++;
    // }
    // else{
    //     speedClampCounter--;
    // }

    // if(speedClampCounter > 1000){
    //     digitalWrite(IN1_Pin,LOW);
    //     digitalWrite(IN2_Pin,LOW);
    // }
    // digitalWrite(IN1_Pin,LOW);
    // digitalWrite(IN2_Pin,HIGH);

}
