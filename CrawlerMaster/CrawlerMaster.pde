//
// CRAWLER MASTER
//
// PVIT RoboMagellan 2010-2011
//
// Written by:  George Kaveladze
//              Zac Peffer
//              Pete Marshall
//
// This module is the main controller program for the RoboMagellan robot. It controls the power and steering servos and, as such,
// has complete control over the robot behavoir. It accesses navigation and sensor information from other controllers by requesting
// information over an I2C (Wire Library) interface. This distributed approach was taken so as to allow sensor and GPS information
// to be refreshed continually, without getting delayed by a slow main processing loop.// the master controller.
//
// Operation:
//
//   This module runs in serveral different modes depending on what the state of the robot is at a particular time.
//
//     STARTUP MODE
//    
//     Startup mode is the initial state of the robot after the controller is reset. It forces the other controllers to re-start
//     their processing (so that the whole system stays in synch from a mode perspective), and then waits for the navigation
//     controller to indicate that it has a valid GPS fix. Once a fix is available, the program moves to Systematic Error Mode.
//
//     SYSTEMATIC ERROR MODE
//
//     Before GPS-based navigation can begin, the navigation controller must calculate its systematic error (i.e. the 
//     difference in latitude and longitude between the given and the measured waypoint at the start of the course). In
//     systematic error mode, this controller sends navigation the first waypoint and then waits for navigation to indicate it
//     has made a successful calculation. Once this happens, the system passes the first target waypoint to navigation, and 
//     enters GPS mode.
//
//     GPS MODE
//
//     In this mode, the robot is navigating toward the next waypoint using GPS information. This controller calls navigation
//     to get the latest distance and steering angle, which it uses to make navigational decisions. In each loop, the system 
//     will also poll the sensor controller to check for obstacles. If and obstacle is found, the program switches to obstacle
//     mode.
//
//     Once the robot is within a pre-determined distance from the waypoint, it will switch to camera mode.
//
//     OBSTACLE MODE
//
//     Obstacle mode is set when the sensors detect an obstacle. The appoach to obstacle detection is simple: go backwards
//     randomly steering left or right for a random time; then drive forwards for a random amount of time (avoiding obstacles at
//     the same time of course!). Once an obstacle has been avoided, switch back to GPS mode.
//     
//     Obstacle mode is also called by camera mode if that mode fails to find the target cone. This is essentially a strategy
//     of giving up, creating a new start position and - by going back to GPS mode - trying again.
//
//     CAMERA MODE
//
//     Camera mode is set when the robot gets close to a waypoint. It now uses information from the camera to steer toward the cone,
//     waiting for the touch sensor to indicate contact. Once contact is made, the next waypoint is set, and we call GPS mode.
//
//     If camera mode loses visual contact with the cone, it will call obstacle mode to move randomly and then restart its
//     search.
//
//     COMPLETED MODE
//
//     We are done! Rejoice. Dance wildly in the streets
//
#include <Servo.h>
#include <Wire.h>
#include "CrawlerHeader.h"

// Define power values
#define NO_POWER      95
#define LOW_POWER    105
#define NORMAL_POWER 108
#define MAX_POWER    150
#define BACKWARD      85

// Define steering values
#define STRAIGHT      84
#define LEFT         114
#define RIGHT         54

// Define MODES
int mode;
#define STARTUP_MODE          0
#define GPS_MODE              1
#define OBSTACLE_MODE         2
#define CAMERA_MODE           3
#define SYSTEMATIC_ERROR_MODE 4
#define COMPLETED_MODE        5

// Define course waypoints;
#define NUMBER_OF_WAYPOINTS 7
int waypointNumber = 0;

double target_lats[NUMBER_OF_WAYPOINTS] = {33.778341,33.778394, 33.778528, 33.778394, 33.778114,33.778528, 33.778341};  
double target_lons[NUMBER_OF_WAYPOINTS] = {118.418851,118.419013, 118.418754, 118.419013, 118.418781,118.418754, 118.418851};

// From Navigation Controller
int navigationDistance;   // distance returned from Navigation controller
int navigationAngle;      // angle returned from Navigation controller

// From Sensor Controller
int oldSensorDistance = 0;
int sensorDistance = 0;       // ultrasonic/touch distance in cm.
int sensorFlag = 0;

// Hardware
Servo steerServo, pwrServo;
#define failSafePin 2

//********************************************************************************************************
// SETUP - the action starts here!
//********************************************************************************************************
void setup() {
  
  mode = STARTUP_MODE;
  
  steerServo.attach(7);
  pwrServo.attach(6);
  
  pinMode(failSafePin, INPUT);
  
  //Interrupt attach on pin 2
  attachInterrupt(0, deadSwitchReleased, LOW);
  
  Serial.begin(9600);
  Wire.begin();
  delay(3000);
  
  forceNavigationRestart();  // force navigation controller to restart
}

void deadSwitchReleased() {
  stopRobot();
  Serial.println("*** FAILSAFE MODE ***");
}

//********************************************************************************************************
// Loop - Main processing loop
//********************************************************************************************************
void loop() {
  
  Serial.print("Main Loop: ");

/*
  if (digitalRead(failSafePin) == LOW) {
   stopRobot();
   Serial.println("*** FAILSAFE MODE ***");
   delay(1000);
  }
  */
  
  //else { 
  
  switch (mode) {
    
    // Startup mode - wait until we have a GPS reading
    case STARTUP_MODE:
      Serial.println("Startup Mode");
      getNavigationData();
      if (navigationAngle == 888) {   // got a valid GPS reading?
        sendInitialWaypoint();        // yes - send the initial waypoint...
        mode = SYSTEMATIC_ERROR_MODE; // ...and wait for systematic error to be caluculated
        break;
      }
      delay(1000);                    // no GPS yet, just wait and try again
      break;
      
    // Systematic error mode - wait for GPS to calibrate
    case SYSTEMATIC_ERROR_MODE:
      Serial.println("Waiting Systematic Error Mode");
      getNavigationData();             
      if (navigationAngle == 555) {
        waypointNumber = 1;           // yes - send the first waypoint
        sendWaypoint(waypointNumber); // ...and...
        mode = GPS_MODE;              // ...go into GPS mode
      }
      delay(1000);                    // still calculating, just wait and try again
      break;
      
    // GPS Mode - navigate to next waypoint
    case GPS_MODE:
      Serial.println("GPS Mode");
      getSensorData();
      if (sensorDistance < 80  && sensorDistance != 0) {
        sensorFlag++;
        if(sensorFlag > 0) {
          sensorFlag = 0;
          stopRobot();
          mode = OBSTACLE_MODE;
          break;
        }
      }
      else {
        sensorFlag = 0;
      }
      getNavigationData();
      if (navigationAngle == 555) {
        delay(1000);
        break;
      }
      
      if (navigationAngle == 666) {  // Lost the GPS Signal...
        stopRobot();                 // Stop the robot, wait, then re-try
        delay(1000);
        break;
      }
      if (navigationDistance < 4.0) {    
         mode = CAMERA_MODE;
         break;
      }
      driveRobot();
      delay(250);
      steerServo.write(STRAIGHT);
      break;
      
    // Camera Mode
    case CAMERA_MODE:
      Serial.println("Camera Mode");
      // hack - just move to next waypoint
      waypointNumber++;
      if (waypointNumber >= NUMBER_OF_WAYPOINTS) {
        mode = COMPLETED_MODE;
        break;
      }
      sendWaypoint(waypointNumber);
      mode = GPS_MODE;
      stopRobot();
      delay(3000);  // Hack - give the navigation controller enough time to process next waypoint.
      break;
    
    // Obstacle mode
    case OBSTACLE_MODE:

      {
      Serial.println("Obstacle Mode");
      int randnum = random(0,2);
      if (randnum == 0) {
        steerServo.write(RIGHT);
      } else {
        steerServo.write(LEFT);
      }
      pwrServo.write(BACKWARD);
      randnum = random(750, 2000);
      delay(randnum);
      
      steerServo.write(STRAIGHT);
      //pwrServo.write(LOW_POWER);
      //randnum = random(1000, 3000);
      //delay(randnum);
      
      stopRobot();
      mode = GPS_MODE;
      break;
      }  
      
    // Completed mode - we're done, do nothing
    case COMPLETED_MODE:
      stopRobot();
      Serial.println("Completed Mode");
      delay(5000);
      break;
      
    // Default - shouldn't get here
    default:
      Serial.print("ERROR: invalid mode in loop - ");
      Serial.println(mode);
  }
  printinfo();
  //}
}


//********************************************************************************************************
// Get sensor distance from Sensor controller
//********************************************************************************************************
void getSensorData()
{
  Wire.requestFrom(WIRE_ADDRESS_SENSORS, 1);
  while (Wire.available()) {
    int ch = Wire.receive();
    oldSensorDistance = sensorDistance;
    sensorDistance = ch;
  }
  Serial.print("Received distance from sensor controller = ");
  Serial.println(sensorDistance, DEC);
}
//********************************************************************************************************
// Get navigation data (distance and angle) from Navigation controller
//********************************************************************************************************
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


//********************************************************************************************************
// Force the Navigation controller to restart
//********************************************************************************************************
void forceNavigationRestart()
{
  // If this controller restarts, we need to restart the Navigation controller
  // so that the two systems remain in synch from the perspective of state.
  // We do this by sending an invalid waypoint (999, 999). The Navigation event
  // handler, on receiving this will force a restart back to the initial state.

  Serial.println("Forcing Navigation reset");  
  sendWaypoint(999);
}


//********************************************************************************************************
// Send the initial waypoint to the Navigation Controller
//********************************************************************************************************
void sendInitialWaypoint()
{
  sendWaypoint(0);
}


//********************************************************************************************************
// Send the nth waypoint to the Navigation controller
//********************************************************************************************************
void sendWaypoint(int waypointIndex)
{
  WAYPOINT_STRUCT waypoint_data;
  
  if (waypointIndex == 999) {                             // if forceNavigationRestart() has been called
    waypoint_data.latitude = 999.0;                       // set the lat/long values to force a restart
    waypoint_data.longitude = 999.0;
  } else {                                                // otherwise send the next waypoint
    waypoint_data.latitude  = target_lats[waypointIndex]; 
    waypoint_data.longitude = target_lons[waypointIndex];
  }

  uint8_t* ptr = (uint8_t *) &waypoint_data;
  
  Serial.print("Sending data: ");
  for (int i = 0; i < sizeof(WAYPOINT_STRUCT); i++) {
    Serial.print(ptr[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
  
  Wire.beginTransmission(WIRE_ADDRESS_NAVIGATION);
  Wire.send(ptr, sizeof(WAYPOINT_STRUCT));
  Wire.endTransmission();
  
  Serial.print("Sent waypoint to Navigation: latitude = ");
  Serial.print(waypoint_data.latitude, DEC);
  Serial.print(", longitude = ");
  Serial.println(waypoint_data.longitude, DEC);
}


//********************************************************************************************************
// Make a steering decision    
//********************************************************************************************************
void steer() {
  //Remember steeringAngle is from -90 (left) to 90 (right)
  //Must convert to Servo values : 0 (right) to 180 (left)
}


//********************************************************************************************************
// Print diagnostic information
//********************************************************************************************************
void printinfo()
{
  Serial.print("Distance = ");
  Serial.print(navigationDistance, DEC);
  Serial.print("  Angle = ");
  Serial.print(navigationAngle, DEC);
  Serial.print("  Sensor = ");
  Serial.println(sensorDistance, DEC);
}


//********************************************************************************************************
// Stop the robot
//********************************************************************************************************
void stopRobot()
{
  pwrServo.write(NO_POWER);
}


//********************************************************************************************************
// Drive the robot based on distance and steering angle from Navigation
//********************************************************************************************************
void driveRobot()
{
  Serial.print("Normal Steer = ");
  Serial.print(STRAIGHT - navigationAngle/2, DEC);
  Serial.print("  ");
  
  
  if(navigationDistance < 6.0) {
    Serial.println("Low Power");
    steerServo.write(STRAIGHT - navigationAngle);
    pwrServo.write(LOW_POWER);
  }
  else {
    Serial.println("Normal Power");
    steerServo.write(STRAIGHT - navigationAngle/2);
    pwrServo.write(NORMAL_POWER);
  }
    
}
