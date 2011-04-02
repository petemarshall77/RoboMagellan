
#include <Servo.h>
#include <Wire.h>


#define NO_POWER 95
#define LOW_POWER 105
#define NORMAL_POWER 110
#define MAX_POWER 150
#define BACKWARD 85

#define STRAIGHT 84
#define LEFT 124
#define RIGHT 54

//MODES
int mode;
#define STARTUP_MODE 0
#define GPS_MODE 1
#define OBSTACLE_MOE 2
#define CAMERA_MODE 3

//Wire Addresses
#define GPS 1
#define SENSORS 2



#define NUMBER_OF_WAYPOINTS 6
int waypointNumber = 0;
double target_lats[NUMBER_OF_WAYPOINTS] = {33.778341, 33.778451, 33.778528, 33.778394, 33.778114, 33.778341};  
double target_lons[NUMBER_OF_WAYPOINTS] = {118.418851, 118.419148, 118.418754, 118.419013, 118.418781, 118.418851};


int gps_distance = 0;
int steeringAngle;

int sensor_distance = 0;


Servo steerServo, pwrServo;


void setup() {
  mode = STARTUP_MODE;
  
  steerServo.attach(7);
  pwrServo.attach(6);
  
  Serial.begin(4800);
  
  Wire.begin();
  
  //Testing
  mode = GPS_MODE;
}

void loop() {
 if(mode == CAMERA_MODE) {
  //UNTESTED 
  int cmu1 = digitalRead(1);
  int cmu2 = digitalRead(2);
  
  if(!cmu1 && !cmu2) {
    steerServo.write(LEFT);
    pwrServo.write(BACKWARD);
  }
  else if(!cmu1 && cmu2) {
    steerServo.write(RIGHT);
    pwrServo.write(LOW_POWER);
  }
 else if(cmu1 && !cmu2) {
   steerServo.write(LEFT);
   pwrServo.write(LOW_POWER); 
 } 
 else if(cmu1 && cmu2) {
   steerServo.write(STRAIGHT);
   pwrServo.write(LOW_POWER);
 }
 delay(500);
 }
 
 else if(mode == GPS_MODE) {
     //Untested
   
     //receving
     //request 2 bytes, distance and steeringAngle (I hope this is the right way!)
     Wire.requestFrom(GPS,2);
     gps_distance = (int)Wire.receive();
     steeringAngle = (int)Wire.receive();
     
     if(steeringAngle==777) {
     //sending
     //MASTER must use beginTransmission() to specify to whom he is sending.
     Wire.beginTransmission(GPS);
     char lat[10];
     char lon[10]; 
     dtostrf(target_lats[waypointNumber],10,7,lats);
     dtostrf(target_lons[waypointNumber],10,7,lons);
     Wire.send(lats);
     Wire.send(lons);
     Wire.endTransmission();
     }
 }
 
 else if(mode == OBSTACLE_MODE) {
   //request distance data as 1 byte
   Wire.requestFrom(SENSORS,1);
   sensor_distance = (int)Wire.receive();
   //do stuff
 }
 
 steer();
 delay(500);
}

void steer() {
  //Remember steeringAngle is from -90 (left) to 90 (right)
  //Must convert to Servo values : 0 (right) to 180 (left)
}

