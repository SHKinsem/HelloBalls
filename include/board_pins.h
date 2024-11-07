// Description: This file contains the pin definitions for the ESP32 board.
// Note EA1 and EB1 for motor0 and EA2 and EB2 for motor1 are the encoder pins.
// 36 39 does not support interrupts, but one of the encoder pin must be connected to an interrupt pin.

#define EA1 35  // Encoder A pin for motor0, original 36, swap with EB2 for interrupt
#define EB1 39
#define EA2 34
#define EB2 36  // Encoder B pin for motor1, original 35, swap with EA1 for non interrupt

#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26

#define CAN_RX 27
#define CAN_TX 14

#define INA 12
#define INB 16
