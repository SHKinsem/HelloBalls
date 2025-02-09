#include "board_pins.h"

#include <Arduino.h>
#include <CAN.h>
#include <dji_motors.h>
#include <freertos/FreeRTOS.h>
#include <dc_motors.h>
#include <serial_com.h>

#define DEBUG true
#define TESTMODE true

void onReceive(int);
float parseCoordinate(const String&, char);

dc_motorClass dc_motor[2];

motorClass motors[4] = {
  motorClass(MOTOR1),
  motorClass(MOTOR2),
  motorClass(MOTOR3),
  motorClass(MOTOR4)
};

BallAngleCalculator calculator;
DistanceCalculator ballDistance;

PIDController angleController;
PIDController distanceController;

// Using the FreeRTOS task to control the motor
void taks_can_sender(void *pvParameters);
void task_serial_sender(void *pvParameters);
void task_serial_receiver(void *pvParameters);
void task_motor(void *pvParameters);

void task_led(void *pvParameters);

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  delay(500);
  float PIDs_0[6] = {1.5, 0.02, 0.08, 
                     0.0, 0.0, 0.0},

        PIDs_1[6] = {2, 0.02, 0.08, 
                     0.0, 0.0, 0.0},

        DC_MOTOR_PIDs_0[6] = {1.5, 0.21, 0.05,
                              0, 0, 0},

        DC_MOTOR_PIDs_1[6] = {1.5, 0.21, 0.05,
                              0, 0, 0};


  motors[0].init(CAN_RX, CAN_TX, PIDs_0, onReceive);  
  motors[1].init(CAN_RX, CAN_TX, PIDs_1, onReceive);

  dc_motor[0].init(EA1, EB1, IN1, IN2, INA, 0);
  dc_motor[1].init(EA2, EB2, IN3, IN4, INB, 2);

  dc_motor[0].set_pid(DC_MOTOR_PIDs_0);
  dc_motor[1].set_pid(DC_MOTOR_PIDs_1);

  angleController.initController(DEFAULT_LOOP);
  angleController.setControllerParams(0.55, 0.005, 0.0);
  angleController.maxOutput = 100;

  distanceController.initController(DEFAULT_LOOP);
  distanceController.setControllerParams(1.2, 0.005, 0.0);
  distanceController.maxOutput = 100;

  xTaskCreatePinnedToCore(task_serial_sender, "Serial Sender", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_serial_receiver, "Serial Receiver", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(task_led, "LED", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taks_can_sender, "CAN Sender", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(task_motor, "Motor", 4096, NULL, 4, NULL, 1);

  Serial.println("Setup done!");
}

void loop() {
  // put your main code here, to run repeatedly:
}


void onReceive(int packetSize) {
  int id = CAN.packetId() - 0x201;
  // Check if the id valid
  if(id > 3 || id < 0) return;
  uint8_t rawData[8];
  for(int i = 0; CAN.available(); i++){
    rawData[i] = CAN.read();
    if(i > 7) break;
  }
  motors[id].motorData.parseRawData(rawData);
}


void taks_can_sender(void *pvParameters) {
  vTaskDelay(500);
  while(1){
    int current0 = motors[0].getCurrent;
    int current1 = motors[1].getCurrent;
    CAN.beginPacket(0x200);
    CAN.write(lowByte(current0) << 8 | highByte(current0));
    CAN.write(0x00);
    CAN.write(lowByte(current1) << 8 | highByte(current1));
    CAN.write(0x00);
    CAN.endPacket();
    vTaskDelay(1);
  }
}

void task_serial_sender(void *pvParameters) {
  Serial.println("Task serial started");
  vTaskDelay(500);
  while(DEBUG) {
    Serial.print("Streaming");
    // Serial.print("current:");
    // Serial.print(motors[0].getCurrent/1000.0);
    // Serial.print(", motor0_speed:");
    // Serial.print(motors[0].motorData.speed);
    // Serial.print(", motor1_speed:");
    // Serial.print(motors[1].motorData.speed);
    // Serial.print(", target_speed:");
    // Serial.print(motors[0].targetSpeed);
    // Serial.print(", motor_position:");
    // Serial.print(motors[0].motorData.readablePosition);
    // Serial.print(", Shaft angle:");
    // Serial.print(motors[0].motorData.shaftAngle);
    // Serial.print(", Target position:");
    // Serial.print(motors[0].motorData.absluteEncoder/(3591.0*360.0/187.0)*rotationDistance);
    // Serial.print(", Abslute encoder:");
    // Serial.print(motors[0].motorData.absluteEncoder);
    // Serial.print(", Target position:");
    // Serial.print(motors[0].targetPosition);
    // Serial.print(", Torque:");
    // Serial.print(motors[0].motorData.torque);
    // Serial.print(", debug:");
    // Serial.print(motors[0].debugOutput);
    // Serial.print(", posPID:");
    // Serial.print(motors[0].posPID.getOutput);
    // Serial.print(", speedPID:");
    // Serial.print(motors[0].speedPID.getOutput);
    // Serial.print(", speedPID1:");
    // Serial.print(motors[1].speedPID.getOutput);
    // Serial.print(", motor0_degree:");
    // Serial.print(dc_motor[0].degree);
    // Serial.print(", motor0_speed:");
    // Serial.print(dc_motor[0].get_speed());
    Serial.print(", motor0_target_speed:");
    Serial.print(dc_motor[0].get_target_speed());
    // Serial.print(", motor0_pid_output:");
    // Serial.print(dc_motor[0].speedController.getOutput);
    // Serial.print(", motor1_encoderCount:");
    // Serial.print(dc_motor[1].encoderCount);
    // Serial.print(", motor1_degree:");
    // Serial.print(dc_motor[1].degree);
    // Serial.print(", motor1_speed:");
    // Serial.print(dc_motor[1].get_speed());
    Serial.print(", motor1_target_speed:");
    Serial.print(dc_motor[1].get_target_speed());
    // Serial.print(", motor1_pid_output:");
    // Serial.print(dc_motor[1].speedController.getOutput);
    Serial.println();
    vTaskDelay(100);
  }
  vTaskDelete(NULL);
}

void task_serial_receiver(void *pvParameters) {
  Serial.println("Task serial receiver started");
  vTaskDelay(500);
  String inString;
  float speed = 0;
  int data = 0;
  bool SETTING_FLAG = false;
  // while(TESTMODE) {
  //   if (Serial.available() > 0) {
  //     data = Serial.parseFloat();
  //     if(Serial.read() == '\n') {
  //       dc_motor[0].set_speed(data);
  //       dc_motor[1].set_speed(data);
  //     }
  //   }
  //   vTaskDelay(50);
  // }

  // while(!TESTMODE){
  //   if (Serial.available() > 0) {
  //     speed = Serial.parseFloat();
  //     if(Serial.read() == '\n') {
  //       motors[0].targetSpeed = speed;
  //       motors[1].targetSpeed = -speed;
  //     }
  //   }
  //   vTaskDelay(50);
  // }
  float angle2speed_prop = -1.0;
  float distance2speed_prop = -1.0;
  float angle_output = 0;
  float distance_output = 0;
  float angle = 0;
  float distance = 0;
  uint32_t serialCounter = 530;
  while(1){
    if (Serial.available() > 0){
      if(serialCounter > 0) serialCounter = 0;
      // Serial.println("data received");
      // Read the incoming string
      String data = Serial.readStringUntil('\n');
      // Serial.println(data);
      float x = parseCoordinate(data, 'x');
      float y = parseCoordinate(data, 'y');
      if (x!=0||y!=0){
        distance = ballDistance.calculateDistance(x,y);
        calculator.setCoordinates(x, y);
        angle = calculator.calculateAngle();

        angle_output = angleController.compute(angle, 0);
        distance_output = distanceController.compute(distance, 0);

        dc_motor[0].set_speed(angle_output * -angle2speed_prop + distance_output * distance2speed_prop);
        dc_motor[1].set_speed(angle_output * angle2speed_prop + distance_output * distance2speed_prop);
        motors[0].targetSpeed = 500;
        motors[1].targetSpeed = -500;
      }
    }
    else{
      serialCounter++;
    }

    if(serialCounter > 30 && serialCounter < 230){
      dc_motor[0].set_speed(20);
      dc_motor[1].set_speed(-20);
    }
    else if(serialCounter > 230 && serialCounter < 430){
      dc_motor[0].set_speed(-30);
      dc_motor[1].set_speed(30);
    }
    else if(serialCounter > 430 && serialCounter < 530){
      dc_motor[0].set_speed(20);
      dc_motor[1].set_speed(-20);
    }
    if(serialCounter >= 530){
      serialCounter = 530;
      dc_motor[0].set_speed(0);
      dc_motor[1].set_speed(0);
      motors[0].targetSpeed = 0;
      motors[1].targetSpeed = 0;
    }
    // Serial.println(distance);
    // Serial.println(angle);
    // dc_motor[0].set_speed(20);
    // dc_motor[1].set_speed(20);
    // Serial.print("Speed: ");
    // Serial.println(angle * angle2speed_prop + distance * distance2speed_prop);
    vTaskDelay(5);
  }

}

void task_motor(void *pvParameters) {
  vTaskDelay(500);
  while(1) {
    motors[0].run();
    motors[1].run();
    dc_motor[0].run();
    dc_motor[1].run();
    vTaskDelay(5);
  }
}

void task_led(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  vTaskDelay(1000);
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500);
  }
}

float parseCoordinate(const String& data, char coordType) {
    String coordStr = coordType + String("=");
    int startIndex = data.indexOf(coordStr) + coordStr.length();
    int endIndex = data.indexOf(',', startIndex);
    if (endIndex == -1) { // If ',' is not found, it must be the end of the string
        endIndex = data.length();
    }
    return data.substring(startIndex, endIndex).toFloat();
}