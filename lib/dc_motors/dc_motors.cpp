#include <dc_motors.h>

dc_motorClass* dc_motorClass_ptr;

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

int8_t dc_motorClass::init(int ea_pin, int eb_pin, int in1_pin, int in2_pin, int pwm_pin, int channel){   // Overlaod function: handles the pin input
    dc_motorClass_ptr = this;
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
        attachInterrupt(digitalPinToInterrupt(encoderA_Pin), interuptHandler, CHANGE);
    }
    return state;
}

void IRAM_ATTR dc_motorClass::encoderInterrupt(){
    if (digitalRead(encoderA_Pin) == digitalRead(encoderB_Pin)) {
        encoderCount++;
    } 
    else encoderCount--;
    degree = abs((encoderCount*(360/410)));
}

void dc_motorClass::cal_speed(){
    currTime = millis();
    long passedTime = currTime - prevTime;

    speed = (degree - prevDegree)*1000/passedTime;
    prevDegree = degree;

    // speed = (encoderCount - prevEncoderCount)*1000/passedTime;
    // prevEncoderCount = encoderCount;

    prevTime = currTime;
}

void dc_motorClass::set_pwm(){
    // int PWM_SPEED = this->speedController.compute(speed, target_speed);
    ledcWrite(PWM_CHANNEL, 20);
}

void dc_motorClass::run(){
    cal_speed();
    this->set_pwm();
    digitalWrite(IN1_Pin,HIGH);
}
