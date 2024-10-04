#ifndef DJI_MOTORS_H
#define DJI_MOTORS_H

#include <Arduino.h>
#include <CAN.h>
#include <PID_Controller.h>

// M3508 motor IDs
enum motor_id {
  MOTOR1 = 0x201,
  MOTOR2 = 0x202,
  MOTOR3 = 0x203,
  MOTOR4 = 0x204
};

class motorDataClass {
  public:
    int id;
    float gearRatio = 3591.0/187.0;
    uint16_t dataArray[4];
    int16_t maxCurrent;
    int16_t maxSpeed;
    int16_t speed;
    int16_t torque;
    int16_t position;
    int8_t temperature;
    int16_t loopCounter = 0;
    int16_t lastPosition = 0;
    
    double shaftAngle = 0;
    float absluteEncoder = 0;

    // PID parameters
    double kp; // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain

    float readableSpeed;
    float readableTorque;
    float readablePosition = -1;
    float readableTemperature;

    double getShaftAngle();
    double getAbsluteEncoder();
    void initDataArray();
    void parseRawData(uint8_t rawData[8]);
    void processMotorData();
    void getLoopCount();
};


// Class to handle the motor
class motorClass {
  private:
    void setSpeedPID(double p, double i, double d);
    void setPosPID(double p, double i, double d);
  public:
    float debugOutput;
    motorClass(int id);
    motorDataClass motorData;
    PIDController speedPID;
    PIDController posPID;
    int16_t getCurrent = 0;

    int16_t targetSpeed = 0;
    int16_t targetTorque = 0;
    float targetPosition = -1;
    float stallCurrent = 0;
    float stalledCurrent = 0;

    void setMaxCurrent(int16_t current);
    void setMaxSpeed(int16_t speed);
    void setSpeed(int16_t speed);
  
    void setPos(int16_t pos);

    void setTorque(int16_t torque);
    void setPosSpeed(float pos, int16_t speed);
    void setPosTorque(int16_t pos, int16_t torque);
    void setPosSpeedTorque(int16_t pos, int16_t speed, int16_t torque);
    void stallDetection();
    void init(int CAN_RX, int CAN_TX, float PIDs[], void (*onReceive)(int));
    void run();
};

#endif // DJI_MOTORS_H