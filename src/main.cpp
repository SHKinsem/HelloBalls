#include <Arduino.h>
#include <CAN.h>
#include <dji_motors.h>
#include <freertos/FreeRTOS.h>
#include <dc_motors.h>

#define CAN_RX 27
#define CAN_TX 14
#define DEBUG true
#define TESTMODE false
void onReceive(int);

dc_motorClass dc_motor[2];

motorClass motors[4] = {
  motorClass(MOTOR1),
  motorClass(MOTOR2),
  motorClass(MOTOR3),
  motorClass(MOTOR4)
};


// Using the FreeRTOS task to control the motor
void taks_can_sender(void *pvParameters);
void task_serial_sender(void *pvParameters);
void task_serial_receiver(void *pvParameters);
void task_motor(void *pvParameters);
void task_led(void *pvParameters);
void task_dc_motor(void *pvParameters);

void setup() {
  // put your setup code here, to run once:
  float PIDs_0[6] = {1.5, 0.02, 0.08, 
                     0.0, 0.0, 0.0};
  float PIDs_1[6] = {2, 0.02, 0.08, 
                     0.0, 0.0, 0.0};

  motors[0].init(CAN_RX, CAN_TX, PIDs_0, onReceive);
  motors[1].init(CAN_RX, CAN_TX, PIDs_1, onReceive);

  // dc_motor[0].init_motor();
  
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
  while(DEBUG) {
    // Serial.print("current:");
    // Serial.print(motors[0].getCurrent/1000.0);
    Serial.print(", motor0_speed:");
    Serial.print(motors[0].motorData.speed);
    Serial.print(", motor1_speed:");
    Serial.print(motors[1].motorData.speed);
    Serial.print(", target_speed:");
    Serial.print(motors[0].targetSpeed);
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
    Serial.print(", speedPID:");
    Serial.print(motors[0].speedPID.getOutput);
    Serial.print(", speedPID1:");
    Serial.print(motors[1].speedPID.getOutput);
    Serial.println();
    vTaskDelay(50);
  }
  vTaskDelete(NULL);
}

void task_serial_receiver(void *pvParameters) {
  Serial.println("Task serial receiver started");
  String inString;
  float speed = 0;

  bool SETTING_FLAG = false;
  while(TESTMODE) {
    if (Serial.available() > 0) {
      inString = Serial.readStringUntil('\n');
    }
    if(inString == "SET_PID") {
      Serial.println("Motor ID:");
      SETTING_FLAG = true;
      while(SETTING_FLAG){
        
      }
    }
    inString = "";
    
  }

  while(!TESTMODE){
    if (Serial.available() > 0) {
      speed = Serial.parseFloat();
      if(Serial.read() == '\n') {
        motors[0].targetSpeed = speed;
        motors[1].targetSpeed = -speed;
      }
    }
    vTaskDelay(50);
  }

}

void task_motor(void *pvParameters) {
  while(1) {
    motors[0].run();
    motors[1].run();
    vTaskDelay(1);
  }
}

void task_led(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500);
  }
}