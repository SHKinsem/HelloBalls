#include <serial_com.h>

void serial_command::parse_command(String command){

// Something goes here to parse the command into main_command and flags
// And then store the value together with the name into some variables

}

void serial_command::parse_flags(){
    
}

BallAngleCalculator::BallAngleCalculator() : _x(0), _y(0) {}

void BallAngleCalculator::setCoordinates(int x, int y) {
    _x = x;
    _y = y;
}

int maxX= 1920;  //x size of the screen
int maxY= 480;  // y size of the screen 
float refX = maxX/2.0; //midpoint as reference
float refY = 0;
float mindistancec=100; //the minimum distance for the camera to detect the ball
float minAngle1=-20; //negative angle enable picking the ball when distance is already minimum 
float minAngle2=20; //positive angle enable picking the ball when distance is already minimum 
float distanceRatio = sqrt(maxX*maxX+maxY*maxY);     //逻辑待完善。如果在最边角位置转动了，摄像头就看不到了 

float BallAngleCalculator::calculateAngle() {
    // Reference point (320, 0)

    // Calculate angle in radians
    //球在中线的左边，角度为负
    //球在中线的右边，角度为正
    float angleRadians = atan((_x - refX)/(_y - refY));

    // Convert radians to degrees
    float angleDegrees = angleRadians * (180.0 / PI);

    // Adjust the angle so that right is positive, left is negative
    return angleDegrees;
}

DistanceCalculator::DistanceCalculator():distance(-1){}

float DistanceCalculator::calculateDistance(int x, int y){
    float distance=sqrt(pow((x - refX), 2) + pow((y - refY), 2));
  
    distance =(distance/distanceRatio)*100;

    return distance;
}

