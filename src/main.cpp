#include <Arduino.h>
#include <CAN.h>
#include <m3508.h>

#define CAN_RX 27
#define CAN_TX 14

void onReceive(int);

motorClass motors[4] = {
  motorClass(MOTOR1),
  motorClass(MOTOR2),
  motorClass(MOTOR3),
  motorClass(MOTOR4)
};


float rotationDistance = 8.0;
int maxHeight = 400;  // 400mm
int minHeight = 0;

int rawDataDebug[8] = {0};
// Helper Functions definition
void stallHomming(motorClass&, float, int, int);

// Using the FreeRTOS task to control the motor
void taks_can_sender(void *pvParameters);
void task_serial_sender(void *pvParameters);
void task_serial_receiver(void *pvParameters);
void task_motor(void *pvParameters);
void task_led(void *pvParameters);
void task_stallHandler(void *pvParameters);

void setup() {
  // put your setup code here, to run once:
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
  // Set the motor PID parameters
  motors[0].setSpeedPID(1.5, 0.3, 0.001);  // 0.57, 0.026, 0.013
  motors[0].setPosPID(0.18, 0, 0);
  motors[0].setMaxCurrent(16384);
  motors[0].setMaxSpeed(9500);
  // motors[0].motorData.gearRatio = 3591.0/187.0;
  // motors[0].stallCurrent = 5000;

  // xEventGroup = xEventGroupCreate();
  xTaskCreatePinnedToCore(task_serial_sender, "Serial Sender", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_serial_receiver, "Serial Receiver", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(task_led, "LED", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taks_can_sender, "CAN Sender", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(task_motor, "Motor", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(task_stallHandler, "Stall Handler", 1024, NULL, 1, NULL, 1);
  // vTaskStartScheduler();
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
    // xEventGroupWaitBits(xEventGroup, 0x01, pdTRUE, pdTRUE, portMAX_DELAY);
    int current = motors[0].getCurrent;
    CAN.beginPacket(0x200);
    // Send the current to the motor 1
    // CAN.write(lowByte(current) << 8 | highByte(current));
    CAN.write(highByte(current));
    CAN.write(lowByte(current));
    // Send the rest of the data which will be zero
    CAN.endPacket();
    // xEventGroupSetBits(xEventGroup, 0x02);
    vTaskDelay(20);
  }
}

void task_serial_sender(void *pvParameters) {
  Serial.println("Task serial started");
  while(1) {
    Serial.print(motors[0].getCurrent/1000.0);
    Serial.print(", ");
    // Serial.print(motors[0].motorData.speed);
    // Serial.print(", ");
    // Serial.print(motors[0].targetSpeed);
    // Serial.print(", ");
    // Serial.print(motors[0].motorData.readablePosition);
    // Serial.print(", ");
    // Serial.print(motors[0].motorData.shaftAngle);
    // Serial.print(", ");
    Serial.print(motors[0].motorData.absluteEncoder/(3591.0*360.0/187.0)*rotationDistance);
    Serial.print(", ");
    // Serial.print(motors[0].motorData.absluteEncoder);
    // Serial.print(", ");
    // Serial.print(motors[0].targetPosition);
    // Serial.print(", ");
    Serial.print(motors[0].motorData.torque);
    // Serial.print(", ");
    // Serial.print(motors[0].debugOutput);
    // Serial.print(", ");
    // Serial.print(motors[0].posPID.getOutput);
    // Serial.print(", ");
    // Serial.print(motors[0].speedPID.getOutput);
    // Serial.print(", ");
    Serial.println();
    vTaskDelay(500);
  }
}

void task_serial_receiver(void *pvParameters) {
  Serial.println("Task serial receiver started");
  char inByte = 0;
  int speed = 0;
  int position = 0;

  while(1) {

    if(Serial.available() > 0) {
      speed = Serial.parseInt();
      char inByte = Serial.read();
      Serial.println(inByte);
      if(inByte == '\n') {
        // motors[0].setSpeed(speed);
        stallHomming(motors[0], 1000, maxHeight, speed);
      }
      else if(inByte == ',') {
        position = Serial.parseInt();
        if(position > maxHeight) position = maxHeight;  // Limit the position
        else if(position < minHeight) minHeight = 0;
        motors[0].setPosSpeed(((position / rotationDistance) * 360.0), speed);
        Serial.read();  // Abandon the newline character
      }
      // else if(inByte == 'h'){
      //   stallHomming(motors[0], 1000, maxHeight, 800);
      //   }
    }

    // if(speed!=0) {
    //   Serial.print("Requested speed: ");
    //   Serial.println(speed);
    // }
    // if(position!=0) {
    //   Serial.print("Requested pos: ");
    //   Serial.println(position);
    // }

    vTaskDelay(500);
  }
}
void task_motor(void *pvParameters) {
  while(1) {
    motors[0].run();
    // xEventGroupSetBits(xEventGroup, 0x01);
    vTaskDelay(20);
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

void stallHomming(motorClass &motor, float stallCurrent, int maxHeight, int homingSpeed) {
  motor.motorData.absluteEncoder = maxHeight;
  motor.stallCurrent = stallCurrent;
  motor.setPosSpeed(1, 1000);
}

void task_stallHandler(void *pvParameters) {
  while(1) {
    if(motors[0].isStalled){
      motors[0].motorData.absluteEncoder = 0;
      motors[0].stallCurrent = 0;
      motors[0].isStalled = false;
    }
    vTaskDelay(50);
  }
} 