#include <dji_motors.h>

double motorDataClass::getShaftAngle() {
  int shaftLoopCounter = this->loopCounter%int(this->gearRatio);
  double angle = (loopCounter*360.0 + this->readablePosition);
  if(angle < 0) angle += 360.0*gearRatio;
  this->shaftAngle = angle/gearRatio;
  return angle;
}

double motorDataClass::getAbsluteEncoder() {
  this->absluteEncoder = (loopCounter * 360.0 + this->readablePosition);
  return this->absluteEncoder;
}

void motorDataClass::parseRawData(uint8_t rawData[8]) {
  // parse the raw data from the motor
  for(int i = 0; i < 3; i++) {
    // Shitf the byte to left by: (byte1 << 8) | byte2)
    this->dataArray[i] = ((rawData[2 * i] << 8) | rawData[2 * i + 1]);
  }
  this->dataArray[3] = rawData[6];  // The last byte rawData[7] is NULL

  // Store the data in the class
  this->position = this->dataArray[0];
  this->speed = this->dataArray[1];
  this->torque = this->dataArray[2];
  this->temperature = this->dataArray[3];
  this->getLoopCount();
}

void motorDataClass::processMotorData() {
  // process the motor data
  this->readablePosition = this->dataArray[0] / 8192.0 * 360.0;
  // The speed and torque maybe nagative
  this->readableSpeed = (int16_t)this->dataArray[1] / 100.0;
  this->readableTorque = (int16_t)this->dataArray[2] / 1000.0;

  this->readableTemperature = this->dataArray[3];
}

void motorDataClass::getLoopCount() {
  this->readablePosition = this->dataArray[0] / 8192.0 * 360.0;

  if(this->position - this->lastPosition > 6500) {
    this->loopCounter--;
  } else if(this->position - this->lastPosition < -6500) {
    this->loopCounter++;
  }
  this->lastPosition = this->position;
}

void motorDataClass::initDataArray() {
  // initialize the data array
  for(int i = 0; i < 4; i++) {
    this->dataArray[i] = 0;
  }
}

motorClass::motorClass(int id) {
  this->motorData.id = id;
  this->motorData.initDataArray();
}

void motorClass::setMaxCurrent(int16_t current) {
  this->motorData.maxCurrent = current;
  this->speedPID.maxOutput = current;
}

void motorClass::setMaxSpeed(int16_t speed) {
  this->motorData.maxSpeed = speed;
}

void motorClass::setSpeedPID(double p, double i, double d) {
  this->speedPID.setControllerParams(p, i, d);
}
// The following only change the current value of the motor, which uses pid controller

void motorClass::setSpeed(int16_t speed) {
  this->targetSpeed = speed;
  // this->getCurrent = this->speedPID.compute(this->motorData.speed, this->targetSpeed);
}


void motorClass::init(int CAN_RX, int CAN_TX, float PIDs[], void (*onReceive)(int)) {

  setSpeedPID(PIDs[0], PIDs[1], PIDs[2]);
  // setPosPID(PIDs[3], PIDs[4], PIDs[5]);
  setMaxCurrent(16384);
  setMaxSpeed(9500);

  CAN.setPins(CAN_RX,CAN_TX);
  CAN.onReceive(onReceive);
  delay(1000);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("M3508 motor control started!");
  if (!CAN.begin(100E4)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void motorClass::run() {

  this->getCurrent = this->speedPID.compute(this->motorData.speed, this->targetSpeed);

  if(this->targetSpeed == 0) {
    this->getCurrent = 0;
  }

}