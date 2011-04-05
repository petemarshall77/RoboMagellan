#include <Servo.h>
#include <Wire.h>
#include "CrawlerHeader.h"

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
#define SYSTEMATIC_ERROR_MODE 4
#define COMPLETED_MODE 5

#define NUMBER_OF_WAYPOINTS 6
int waypointNumber = 0;
double target_lats[NUMBER_OF_WAYPOINTS] = {33.778341, 33.778451, 33.778528, 33.778394, 33.778114, 33.778341};  
double target_lons[NUMBER_OF_WAYPOINTS] = {118.418851, 118.419148, 118.418754, 118.419013, 118.418781, 118.418851};


int navigationDistance;   // distance returned from Navigation controller
int navigationAngle;      // angle returned from Navigation controller

Servo steerServo, pwrServo;


void setup() {
  
  mode = STARTUP_MODE;
  
  steerServo.attach(7);
  pwrServo.attach(6);
  
  Serial.begin(9600);
  Wire.begin();
}


void loop() {
  
  Serial.print("Main Loop: mode = ");
  Serial.println(mode, DEC);
  
  switch (mode) {
    
    // Startup mode - wait until we have a GPS reading
    case STARTUP_MODE:
      getNavigationData();
      if (navigationAngle == 888) {   // got a valid GPS reading?
        sendInitialWaypoint();
        mode = SYSTEMATIC_ERROR_MODE;
        break;
      }
      delay(1000);
      break;
      
    // Systematic error mode - wait for GPS to calibrate
    case SYSTEMATIC_ERROR_MODE:
      getNavigationData();
      if (navigationAngle == 555) {
        waypointNumber = 1;
        sendWaypoint(waypointNumber); 
        mode = GPS_MODE;
      }
      delay(1000);
      break;
      
    // GPS Mode - navigate to next waypoint
    case GPS_MODE:
      getNavigationData();
      if (navigationAngle == 666) {
        stopRobot();
        delay(1000);
        break;
      }
      if (navigationDistance < 3.0) {
         waypointNumber++;
         if (waypointNumber >= NUMBER_OF_WAYPOINTS) {
           stopRobot();
           mode = COMPLETED_MODE;
           break;
         }
      }
      driveRobot();
      break;
      
    // Completed mode - we're done, do nothing
    case COMPLETED_MODE:
      delay(5000);
      break;
      
    // Default - shouldn't get here
    default:
      Serial.print("ERROR: invalid mode in loop - ");
      Serial.println(mode);
  }
}


// Get navigation data (distance and angle) from Navigation controller
void getNavigationData()
{
  NAV_STRUCT nav_data;

  uint8_t* ptr = (uint8_t *) &nav_data;
  char ch;
  
  Wire.requestFrom(WIRE_ADDRESS_NAVIGATION, sizeof(NAV_STRUCT));
  
  while (Wire.available()) {
     ch = Wire.receive();
     *ptr++ = ch;
  }
  
  navigationDistance = nav_data.distance;
  navigationAngle = nav_data.angle;
  
  Serial.print("Received data from Navigation: distance = ");
  Serial.print(navigationDistance,DEC);
  Serial.print(", angle = ");
  Serial.println(navigationAngle, DEC);
}


// Send the initial waypoint to the Navigation Controller
void sendInitialWaypoint()
{
  sendWaypoint(0);
}


// Send the nth waypoint to the Navigation controller
void sendWaypoint(int waypointIndex)
{
  WAYPOINT_STRUCT waypoint_data;
  
  waypoint_data.latitude  = target_lats[waypointIndex];
  waypoint_data.longitude = target_lons[waypointIndex];

  uint8_t* ptr = (uint8_t *) &waypointIndex;
  
  Wire.beginTransmission(WIRE_ADDRESS_NAVIGATION);
  Wire.send(ptr, sizeof(WAYPOINT_STRUCT));
  Wire.endTransmission();
  
  Serial.print("Sent waypoint to Navigation: latitude = ");
  Serial.print(waypoint_data.latitude, DEC);
  Serial.print(", longitude = ");
  Serial.println(waypoint_data.longitude, DEC);
}
    

  
void steer() {
  //Remember steeringAngle is from -90 (left) to 90 (right)
  //Must convert to Servo values : 0 (right) to 180 (left)
}


// Stop the robot
void stopRobot()
{
  pwrServo.write(NO_POWER);
}


// Drive the robot based on distance and steering angle from Navigation
void driveRobot()
{
  // implement
}
