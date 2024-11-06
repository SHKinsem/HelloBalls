#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <Arduino.h>

#define MAX_COMMAND_SIZE 24
#define MAX_FLAG_SIZE 24
#define MAX_FLAGS 5

class flag{
    String flag_name[MAX_FLAG_SIZE];
    float flag_value;
};

class serial_command {
private:
    String main_command[MAX_COMMAND_SIZE];
    flag flags[MAX_FLAGS];
public:
    void parse_command(String command);
    void parse_flags();

    String get_main_command_name();
    flag get_flag_value(String flag_name);
    
};

float maxX= 1920;  //x size of the screen
float maxY= 480;  // y size of the screen 
float refX = maxX/2; //midpoint as reference
float refY = 0;
float mindistancec=100; //the minimum distance for the camera to detect the ball
float minAngle1=-20; //negative angle enable picking the ball when distance is already minimum 
float minAngle2=20; //positive angle enable picking the ball when distance is already minimum 

class BallAngleCalculator {
  private:
    float _x, _y;

  public:
    BallAngleCalculator(); // Constructor
    void setCoordinates(float x, float y);
    float calculateAngle();
};

class DistanceCalculator{
  private:
      float distance;

  public:
    DistanceCalculator();

    float calculateDistance(float x,float y);
  
};

#endif  // SERIAL_COM_H