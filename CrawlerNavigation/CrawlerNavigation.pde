//
// CRAWLER NAVIGATION
//
// PVIT RoboMagellan 2010-2011
//
// Written by:  George Kaveladze
//              Zac Peffer
//              Pete Marshall
//
// This module runs on the GPS controller. Its role is to read and parse data from a GPS unit (using the standard NMEA serial
// interface), and to calculate and return a distance and angle to a given waypoint. Waypoints are provided to this module by
// the master controller.
//
// Operation:
//
//    The main loop in this module reads the current GPS data, parses the $GRMC sentence to determine the current latitude, longitude,
//    and track-angle (the direction the robot is moving in). Using these data, it calculates the distance and steering-angle between
//    the current GPS position (i.e. where the robot currently thinks it is) and a target waypoint that has been provided by the
//    master controller.
//
//    Distance will be returned in meters, and the steering angle is the angle (from -180 degrees to +180 degrees) from the current
//    robot heading to the waypoint.
//
//    The distance and steering angle values are computed once per loop and stored in variables delta_distance and steeringAngle.
//
//    To receive the current distance and waypoint, the master will ask for the data (using Wire.RequestFrom) from this
//    module. The data will be returned in a NAV_STRUCT structure.
//
//    There are a number of cases where the distance and angle cannot be returned:
//
//      * When the system has just been started and the GPS does not have a fix. In this case, this module will return a NAV_STRUCT
//        structure with the angle set to 999.
//
//      * When the system has a GPS fix but but needs the starting waypoint in order to calculate the systematic GPS error. In this
//        case, an angle of 888 will be returned. On receiving an 888, the master should send the first waypoint. Once this is
//        received, this module will return an angle of 777 until the first true target waypoint is received.
//      
//      * If the GPS signal is lost, this module will return an angle of 666
//
//    Receiving New Waypoints:
//
//      In normal operation, this module will return data to the master controller on demand - i.e. the master issues a Wire.RequestFrom
//      and this module returns data using Wire.Send() in the wireReceiveEvent() handler. When the master wants to send a new waypoint,
//      it does so by writing the data with beginTransmission()/Send()/endTransmission() and this module receives the data in its
//      wireRequestEvent() handler.
//
//
//
//  Arduino Controller Pin Assignments:
//
//    0 - arduino serial Rx (USB)
//    1 - arduino serial Tx (USB)     
//    4 - I2C SDA line - for Wire Library
//    5 - I2C SCL line - for Wire Library
//    6 - GPS transmit - uses NewSoftwareSerial
//    7 - GPS receive  - uses NewSoftwareSerial
//

//  Needed Libraries
#include <NewSoftSerial.h>
#include <Wire.h>
#include "CrawlerHeader.h"

//  Declarations
#define GPS_TX_PIN 3
#define GPS_RX_PIN 2

#define GPS_SERIAL_RATE     4800
#define MONITOR_SERIAL_RATE 9600

#define EARTH_RADIUS        6378100       // in meters, and we'll assume the Earth is a perfect sphere :)
#define PI                  3.141592654   // should be enough decimal places!

// Serial ports
NewSoftSerial gpsSerial = NewSoftSerial(GPS_RX_PIN, GPS_TX_PIN);

double currentBearing = 0;     // current bearing to waypoint
double floatTrackangle;        // vehicle direction in degrees

// GPS parser variables
#define GPS_BUFFER_SIZE 90     // buffer size, 90 bytes is plenty
char buffer[GPS_BUFFER_SIZE];  // buffer for GPS strings
char *parseptr;                // pointer into buffer as we parse
char buffidx;
  
char gpsStatus;                // GPS status: V == Invalid, A == Valid
char latdir, longdir;          // latitude, longitude hemispheres - N/S, E/W

double actual_lat, actual_lon; // Actual position of robot, in degrees
double targetWaypointLatitude; // Target latitude
double targetWaypointLongitude;// Target longitude

double delta_lat, delta_lon;   // Difference in latitude, longitude between actual and target position
double delta_lat_avg;          // Time series average of latitude readings
double delta_lon_avg;          // Time series average of longitude readings

double delta_lat_meters;       // Distance (in meters) north/south between actual and target position
double delta_lon_meters;       // Distance (in meters) east/west between actual and target position
double delta_distance;         // Distance (in meters) betwenn actual and target position <<-- this is a key return value

boolean gpsCalibrated = false; // GPS is calibrated for systematic error (true/false)
double syst_err_lat;           // Systematic error in latitude
double syst_err_lon;           // Systematic error in longitude

int steeringAngle;             // Angle betwen current bearing and target waypoint <<-- this is key return value

int processState;              // 999 - initial, 888 - calculate systematic error,
                               // 777 - waiting start waypoint, 666 - no GPS signal, 555 - waiting for 1st target waypoint 

bool forceRestart = false;     // if set to true, will force a restart

char compileTime[] = __TIME__;
char compileDate[] = __DATE__;

//********************************************************************************************************
// SETUP - the action starts here!
//********************************************************************************************************
void setup() 
{ 
  
  // Set pin statuses as needed
  pinMode(GPS_RX_PIN, INPUT);
  pinMode(GPS_TX_PIN, OUTPUT);
  
  // Start communication
  Serial.begin(MONITOR_SERIAL_RATE);      // arduino --> serial port monitoring (via USB cable)
  gpsSerial.begin(GPS_SERIAL_RATE);       // GPS shield <--> arduino

  Wire.begin(WIRE_ADDRESS_NAVIGATION);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent); 
 
  // Show we're alive 
  Serial.println("PVIT ROBOMAGELLAN CrawlerNavigation");
  Serial.print("Compiled at ");
  for (int i = 0; i < strlen(compileTime); i++) {
     Serial.print(compileTime[i]);
  }
  Serial.print(" on ");
  for (int i = 0; i < strlen(compileDate); i++) {
    Serial.print(compileDate[i]);
  }
  Serial.println("");  

  processState = 999;                    // We're in initial state
  targetWaypointLatitude = 0.0;          // Set to zero - don't have Waypoint
  targetWaypointLongitude = 0.0;         // Set to zero - don't have Waypoint
}


//********************************************************************************************************
// LOOP - Main processing loop
//********************************************************************************************************
void loop() 
{ 
  // Check for a force restart
  if (forceRestart) {
       Serial.println("!!! Reset forced !!!");
       processState = 999;
       forceRestart = false;
       delay(500);    // give the calling controller a few seconds to get started
  }
  
  Serial.print("Main Loop: ");
     
  parseLine();   // Get the GPS Reading
     
  switch (processState) {
       
    // Initial state
    case 999:
       Serial.println(" Waiting for GPS Signal");
       if (gpsStatus == 'A') {     // If we get a valid GPS reading
           processState = 888;     // go to next process state
       }
       break;
         
    // Waiting to calculate GPS systematic error  
    case 888:
      Serial.println("Waiting for initial waypoint");
      if (targetWaypointLatitude != 0.0) {  // If we have received the initial waypoint
         getSystematicError();               // get a reading for the start point
         processState = 555;                 // and go to next process state
      }
      break;
       
    // Processing waypoints
    case 777:
      Serial.println("Processing waypoint");
      if (targetWaypointLatitude != 0.0) { // If we have a target waypoint
        if (gpsStatus == 'A') {            // and we have a valid GPS reading
          calculatePosition();             // calculate a distance and angle to target
        } else {
          processState = 666;              // Invalid GPS - go to 666 state
        }
      } else {                             // No waypoint provided
        processState = 555;                // wait for waypoint
      }
      break;
      
    // Invalid GPS reading state
    case 666:
      Serial.println("Invalid GPS Reading");
      if (gpsStatus == 'A') {    // If we have a valid GPS reading
        calculatePosition();     // calculate a distance and angle to target
        processState = 777;      // and go back regular status
      }
      break;
      
    // Waiting for a waypoint
    case 555:
      Serial.print("Waiting for next waypoint");
      if (targetWaypointLatitude != 0) {   // have we got a target now?
        processState = 777;                // yes - go to normal status
      }
      break;
       
    // Shouldn't get here!!!  
    default:
      Serial.print("ERROR: invalid processState: ");
      Serial.print(processState, DEC);
  }
}


//********************************************************************************************************
// Calculate the GPS systematic error
//********************************************************************************************************
void getSystematicError()
{
  //  Wait for the master to send the initial waypoint. Once we have that, get 10 good GPS readings,
  //  calculate the average latitude and longitude, and subtract from provided latitude and longitude
  //  to calculate systematic error.
  
  while (targetWaypointLatitude == 0.0) {
    delay(100);
  }
  
  int num_readings = 0;
  double lat_sum = 0.0;
  double lon_sum = 0.0;
   
  // get 10 good readings - we throw these 1st 10 away to allow the GPS to settle
  while(num_readings < 10) {
    parseLine();
    if(actual_lat != 0) {
       num_readings++;
    }
    else {
    num_readings = 0;
    }
  }
  
  // now get 10 more readings and sum the latitude and longitude 
  num_readings = 0;
  while(num_readings < 10) {
    parseLine();
    if(actual_lat != 0) {
       num_readings++;
       lat_sum += actual_lat;
       lon_sum += actual_lon;
     }
     else {
       num_readings = 0;
       lat_sum = 0;
       lon_sum = 0;
     }
   }

   //calculate (and print) GPS systematic error
   syst_err_lat = lat_sum/10.0 - targetWaypointLatitude;
   syst_err_lon = lon_sum/10.0 - targetWaypointLongitude;
  
   
   Serial.println("GPS Systematic Error");
   Serial.print("Lat:");
   Serial.println(syst_err_lat,DEC);
   Serial.print("Lon:");
   Serial.println(syst_err_lon,DEC);
   
   // Indicate we now need first target waypoint
   targetWaypointLatitude = targetWaypointLongitude = 0.0;
   
}


//********************************************************************************************************
// Calculate the distance and angle to target
//********************************************************************************************************
void calculatePosition()
{
  // Call calculateDeltas() to return the differences between target and actual positions. Then get the
  // current bearing, and finally steeringAngle - the angle between the current bearing and the target.

  calculateDeltas();
  currentBearing = fixAngle(calculateBearing()); 
  steeringAngle = fixAngle(currentBearing - floatTrackangle);
  
  printInfo();  // print debugging information
}

// Ensure angle returned is -180 < angle < 180
double fixAngle(double angle) {
  
  if(abs(angle) > 180) {
    if(angle < 0) {
      return 360 + angle;
    }
    return angle-360;  
  }
  return angle;
}


//********************************************************************************************************
// Calculate the bearing to the target
//********************************************************************************************************
double calculateBearing() {

  // Calculate bearing from current position to next waypoint
  // 2 steps:
  //     1) calculate an angle using basic trig
  //     2) fix up the angle based on which quadrant the lattitude and longitude are in
  
  double bearing = atan(delta_lat_meters/delta_lon_meters) * (180/PI);
       
  if((targetWaypointLatitude == actual_lat) && (targetWaypointLongitude > actual_lon))
    bearing = 270;
      
  if((targetWaypointLatitude == actual_lat) && (targetWaypointLongitude < actual_lon))
    bearing = 90;

  if((targetWaypointLatitude > actual_lat) && (targetWaypointLongitude == actual_lon))
    bearing = 0;
          
  if((targetWaypointLatitude < actual_lat) && (targetWaypointLongitude == actual_lon))
    bearing = 180;

  if((targetWaypointLatitude > actual_lat) && (targetWaypointLongitude > actual_lon))
    bearing = bearing + 270;

  if((targetWaypointLatitude < actual_lat) && (targetWaypointLongitude > actual_lon))
    bearing = 270 - bearing;

  if((targetWaypointLatitude > actual_lat) && (targetWaypointLongitude < actual_lon))
    bearing = 90 - bearing;

  if((targetWaypointLatitude < actual_lat) && (targetWaypointLongitude < actual_lon))
    bearing = bearing + 90;
    
  return bearing; 
}

//********************************************************************************************************
// Calculate the delta distances between the actual and target latitudes
//********************************************************************************************************
void calculateDeltas() {
  
  //  1. calculate difference between target and current lat and lon.
  //  2. Convert from degrees to meters  
  //  3. Find averages, and use pythagoras to find distance in meters between target position and current position.
     
     
  // Calculate distance and bearing to target
  delta_lat = abs(targetWaypointLatitude - (actual_lat - syst_err_lat));
  delta_lon = abs(targetWaypointLongitude - (actual_lon - syst_err_lon));

  delta_lat_meters = EARTH_RADIUS * (PI / 180.0) * delta_lat;
  delta_lon_meters = EARTH_RADIUS * (PI / 180.0) * delta_lon  * sin((90-actual_lat)*(PI/180));

  // Time average the deltas smooth variations in GPS readings
  // REMOVED
  /*------ 
      for(int i =0; i<4;i++) {
        delta_lats[i] = delta_lats[i+1];
        delta_lons[i] = delta_lons[i+1];
      }

      delta_lats[4] = delta_lat_meters;
      delta_lons[4] = delta_lon_meters;

      delta_lat_avg = (delta_lats[0] + delta_lats[1] + delta_lats[2] + delta_lats[3] + delta_lats[4])/5.0;
      delta_lon_avg = (delta_lons[0] + delta_lons[1] + delta_lons[2] + delta_lons[3] + delta_lons[4])/5.0;
  
  ------*/
      
  // (no averaging, so just use last value)
  delta_lat_avg = delta_lat_meters;
  delta_lon_avg = delta_lon_meters;

  // distance, by Pythagoras
  delta_distance = sqrt(sq(delta_lat_avg) + sq(delta_lon_avg));
}


//********************************************************************************************************
// Parse a string decimal into an int
//********************************************************************************************************
uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  while (str[0] != 0) {
    if ((str[0] > '9') || (str[0] < '0'))
      return d;
    d *= 10;
    d += str[0] - '0';
    str++;
  }
  return d;
}


//********************************************************************************************************
// Read a line of GPS data into the input buffer
//********************************************************************************************************
void readline(void) {

  char c;
  buffidx = 0; // start at begninning
  while (1) {
      c=gpsSerial.read();
      if (c == -1)
        continue;
//      Serial.print(c);   // debugging  
      if (c == '\n')
        continue;
      if ((buffidx == GPS_BUFFER_SIZE-1) || (c == '\r')) {
        buffer[buffidx] = 0;
        return;
      }
      buffer[buffidx++]= c;
  }
}


//********************************************************************************************************
// Parse the GPS $GRMC sentence
//********************************************************************************************************
void parseLine() {
  
  //  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  //  
  //  Where:
  //     RMC          Recommended Minimum sentence C
  //     123519       Fix taken at 12:35:19 UTC
  //     A            Status A=active or V=Void.
  //     4807.038,N   Latitude 48 deg 07.038' N
  //     01131.000,E  Longitude 11 deg 31.000' E
  //     022.4        Speed over the ground in knots
  //     084.4        Track angle in degrees True
  //     230394       Date - 23rd of March 1994
  //     003.1,W      Magnetic Variation
  //     *6A          The checksum data, always begins with *

  //reset the buffer
  buffer[0] = 'x';
 
 // check if $GPRMC (global positioning fixed data)
  while (strncmp(buffer, "$GPRMC",6) != 0) {
    readline();
}
  // DEBUG - print the line
  Serial.print("GPS: ");
  Serial.println(buffer);
        
    
  parseptr = buffer+7; //skip to date/time info
  skipFields(1);       // skip to status info
    

  // GPS status (V = valid, A = invalid)
  gpsStatus = parseptr[0];
  parseptr += 2;                          // skip to latitude info

  // parse latitude data
  actual_lat = parseLatLon();
    
  if(actual_lat == 0) {
    actual_lon = 0;
    Serial.println("No Reading");
    return;   
  }

  skipFields(1);  // skip to N/S info

  // read latitude N/S data
  if (parseptr[0] != ',') {
    latdir = parseptr[0];
  }
  skipFields(1);  // skip to longitude info

  // parse longitude data 
  actual_lon = parseLatLon();
           
  skipFields(1);   // skip to E/W info

  // read longitude E/W data
  if (parseptr[0] != ',') {
    longdir = parseptr[0];
  }
  skipFields(1);   // skip to groundspeed info

  //we  ignore this info
  skipFields(1);   // skip to trackangle info

  // Parse track angle
  //TODO: Check that GPS spec is 1 decimal place.
  int trackangle = parsedecimal(parseptr);
  trackangle *= 10;
  parseptr = strchr(parseptr, '.') + 1;   // skip to decimal part of number 
  trackangle += parsedecimal(parseptr);
    
  floatTrackangle = (float)trackangle/10.0;
    
  skipFields(1);     // skip to next field - ignored
}

// Skip to next field
void skipFields(int n) {
  
  for(int i = 0;i < n; i++) {
    parseptr = strchr(parseptr, ',') + 1;
  } 
}

// Parse the GPS latitude and longitude fields
double parseLatLon() {
    
  // l = latitude or longitude at the current parseptr position
  int l = parsedecimal(parseptr);

  if(l == 0) {
      return 0;
  }
    
  int actual_l_deg = l/100;
  double actual_l_min = l % 100;
   
  parseptr = strchr(parseptr, '.') + 1; // skip to decimal minutes data
  actual_l_min += parsedecimal(parseptr)/10000.0;
    
  return actual_l_deg + actual_l_min/60.0;
}


//********************************************************************************************************
//  Print diagnostic information
//********************************************************************************************************
void printInfo() {

  Serial.print("Lat: "); 
  Serial.print(actual_lat,DEC);
  Serial.print("  ");
  Serial.print("Long: ");
  Serial.println(actual_lon,DEC); 

  Serial.print("Delta Latitude in meters: ");
  Serial.print(delta_lat_meters);
  Serial.print("  ");
  Serial.print("Delta Longitude in meters: ");
  Serial.print(delta_lon_meters);
  Serial.print("  ");  
  Serial.print("Delta distance: ");
  Serial.println(delta_distance);
 
  Serial.print("Vehicle track angle: ");
  Serial.print(floatTrackangle);
  Serial.print("  ");
  Serial.print("Bearing to target: ");  
  Serial.print(currentBearing);
  Serial.print("  ");
  Serial.print("Steering Angle: ");
  Serial.println(steeringAngle);   

}



//********************************************************************************************************
// Send navigation data to master controller
//********************************************************************************************************
void requestEvent() {
  
  //  This routine is called when ever the master controller requests distance and steering 
  //  information. We return a structure containing that information.
  
  NAV_STRUCT navigation_data;
  
  if (processState == 777) {
    navigation_data.distance = delta_distance;
    navigation_data.angle = steeringAngle;
  } else {
    navigation_data.distance = 0.0;
    navigation_data.angle = processState;
  }
  
  uint8_t* ptr = (uint8_t *) &navigation_data;
  Wire.send(ptr, sizeof(NAV_STRUCT));
  }
   
  
//********************************************************************************************************
// Receive waypoint data from master controller
//********************************************************************************************************
void receiveEvent(int receivedByteCount) {

  //  This routine is called when the master sends new waypoint data. 
  
  WAYPOINT_STRUCT received_waypoint;
 
  if (receivedByteCount != sizeof(WAYPOINT_STRUCT)) {
    Serial.println("ERROR: receiveEvent - invalid number of bytes recieved");
    return;
  }
  
  uint8_t* ptr = (uint8_t *) &received_waypoint;
  char ch;

  Serial.print("Receiving data: ");
  while (Wire.available()) {
    ch = Wire.receive();
    *ptr++ = ch;
    Serial.print(ch, HEX);
    Serial.print(" ");
  }
  Serial.println("");
 
  if ((received_waypoint.latitude > 998.0) && (received_waypoint.longitude > 998.0)) { // Master forcing a restart
    forceRestart = true;
    processState = 999;  // force this, so that onRequest sends initial value between now and when forceRestart is
                         // tested. 
    return;
  }
  
  targetWaypointLatitude = received_waypoint.latitude;
  targetWaypointLongitude = received_waypoint.longitude;
  
  Serial.print("Received new waypoint from master: latitude = ");
  Serial.print(targetWaypointLatitude, DEC);
  Serial.print(", longitude = ");
  Serial.println(targetWaypointLongitude, DEC);

    
}
      

