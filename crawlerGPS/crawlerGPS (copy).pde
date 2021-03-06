*
Notes:
 *current version uses floatTrackAngle, which is provided by GPS, to calculate steering angle
 *once I understand how parseDecimal() works, I can probably trim some more lines off the code
*/

// Robomagellan Version 5
// Given a series of GPS waypoints, drive the vehicle to each in turn.

// Arduino Controller Pin Assignments
//  0 - arduino serial Rx (USB)
//  1 - arduino serial Tx (USB)
//  2 - GPS shield, serial in
//  3 - GPS shield, serial out
//  4 - GPS shield, on/off - set low to turn GPS on
//
#define gpsRxPin  2
#define gpsTxPin  3
#define gpsPwrPin 4
#define pingPin 7
#define NO_POWER 95
#define LOW_POWER 105
#define NORMAL_POWER 110
#define MAX_POWER 150
#define BACKWARD 85

#define STRAIGHT 84

// Useful constants
//  GPSRATE - baud rate for communicating with GPS shield
//  EARTH_RADIUS - in meters (this application can safely assume a perfect sphere)
//  PI - no more than adequate decimal places
#define GPSRATE 4800
#define EARTH_RADIUS 6378100
#define PI 3.141592654

// Needed libraries
#include <SoftwareSerial.h>
#include <Servo.h>

// Serial and servo ports
SoftwareSerial gpsSerial = SoftwareSerial(gpsRxPin,gpsTxPin);
Servo pwrServo, steerServo;

// Define course waypoints
//  Assumption: program will run in Northern Hemisphere and West of Prime Meridian - i.e. GPS indicators will be N and W - so we
//  don't include them in the course definition.
//  *** test track around PVHS blacktop outside room 501 ***

// double target_lats[NUMBER_OF_WAYPOINTS] = {33.77831,  33.77843,  33.77806,  33.77786,  33.77841,  33.77855,  33.77835};
// double target_lons[NUMBER_OF_WAYPOINTS] = {118.41883, 118.41902, 118.41920, 118.41863, 118.41833, 118.41878, 118.41887};
//double target_lats[NUMBER_OF_WAYPOINTS] = {33.77840000, 33.77857500, 33.77844444, 33.77809722, 33.77796944, 33.77825833, 33.77840000};
//double target_lons[NUMBER_OF_WAYPOINTS] = {118.41884722, 118.41871111, 118.41898333, 118.41925000, 118.41878889, 118.41858333,118.41884722};
//Z'OLD COURSE 1-9-11
//double target_lats[NUMBER_OF_WAYPOINTS] = {33.77845,33.77845,33.77809,33.77791,33.77840,33.77856,33.77836};
//double target_lons[NUMBER_OF_WAYPOINTS] = {118.41884,118.41898,118.41918,118.41852,118.41836,118.41876,118.41891};

//STRAIGHTAWAY
//double target_lats[NUMBER_OF_WAYPOINTS] = {33.77837,33.77842,33.77837};
//double target_lons[NUMBER_OF_WAYPOINTS] = {118.41882,118.41897,118.41882};

//Football Field [East to West]
//double target_lats[NUMBER_OF_WAYPOINTS] = {33.7779,33.7781};
//double target_lons[NUMBER_OF_WAYPOINTS] = {118.41915,118.42005};


#define NUMBER_OF_WAYPOINTS 6
double target_lats[NUMBER_OF_WAYPOINTS] = {33.778341, 33.778451, 33.778528, 33.778394, 33.778114, 33.778341};  
double target_lons[NUMBER_OF_WAYPOINTS] = {118.418851, 118.419148, 118.418754, 118.419013, 118.418781, 118.418851};

int waypointNumber = 0;     // current waypoint
double currentBearing = 0;  // current bearing to waypoint
double floatTrackangle;     // vehicle direction in degrees

// GPS parser variables
#define BUFFSIZ 90     // buffer size, 90 bytes is plenty
char buffer[BUFFSIZ];  // buffer for GPS strings
char *parseptr;        // pointer into buffer as we parse
char buffidx;

//GPS status (V == Invalid, A == Valid)
char status,latdir,longdir;

// Actual position
double actual_lat, actual_lon;                                            // in degrees

// Last N readings to average deltas in GPS readings (reduces noise)
double delta_lats[5];
double delta_lons[5];
double delta_lat_avg, delta_lon_avg;

// Current delta from last position
 double delta_lat, delta_lon;                                              // in degrees

// Delta in position in meters
double delta_lat_meters, delta_lon_meters, delta_distance;

//GPS systematic error
double syst_err_lat;
double syst_err_lon;


int steeringAngle;


int pingDistance;

long time = 0;


// SETUP - the action starts here!
void setup() 
{ 
  // Set pin statuses as needed
  pinMode(gpsPwrPin, OUTPUT);     // set to output, need to set low to turn GPS on   

  // Start communication
  Serial.begin(GPSRATE);          // arduino --> serial port monitoring (via USB cable)
  gpsSerial.begin(GPSRATE);       // GPS shield <--> arduino

  // Set up power and steering controls
  
 
  steerServo.attach(5);  
  pwrServo.attach(6);

      
  steerServo.write(STRAIGHT);
  pwrServo.write(NO_POWER);

//temp -- George's house 
//target_lats[0] = 33.77765;
//target_lons[0] = 118.40942;


  // Show we're alive 
  Serial.println("ROBOMAGELLAN: Version 4"); 
  
  
  // Turn GPS shield on
  digitalWrite(gpsPwrPin, LOW);

  
  //wait for 10 good readings
   int num_readings = 0;
   double lat_sum = 0;
   double lon_sum = 0;
   
   //get 10 good readings
   while(num_readings < 10) {
     parseLine();
     if(actual_lat != 0) {
       num_readings++;
     }
     else {
     num_readings = 0;
     }
   } 
   num_readings = 0;
   
   //average next 10 good readings
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
   //TODO: Should we average out several readings?
   syst_err_lat = lat_sum/10.0 - target_lats[0];
   syst_err_lon = lon_sum/10.0 - target_lons[0];
   waypointNumber=1;
   
   Serial.println("GPS Systematic Error");
   Serial.print("Lat:");
   Serial.println(syst_err_lat,DEC);
   Serial.print("Lon:");
   Serial.println(syst_err_lon,DEC);
   

} // end of setup 




// MAIN LOOP STARTS HERE

void loop() 
{ 
  
    // If the course is completed, do nothing! 

    if (waypointNumber == NUMBER_OF_WAYPOINTS )
    {
      Serial.println("*** Course Completed ***");
      pwrServo.write(NO_POWER);
      steerServo.write(STRAIGHT);
      delay(10000);  
    }
    else
    {
      
       startTimer();   
      
      //parseLine() returns 
      //actualLat, actualLon, floatTrackAngle, status, latDir, lonDir
      
      parseLine();
      
      Serial.print("GPS Time: ");
      Serial.println(stopTimer()/1000.0);
 
      
      //calculateDelta() returns
      //delta_lat,delta_lon
      //delta_lat_meters,delta_lon_meters
      //delta_lat_avg,delta_lon_avg,delta_distance      
      
      startTimer();
      
      checkForObstacles();
      
      calculateDeltas();
     
     
      
      currentBearing = fixAngle(calculateBearing());
   
      
            //DISABLED: Save old bearing -- using GPS-determined bearing instead. 
      //double oldBearing = currentBearing;

                       
   
      // Calculate a steering angle - the degrees right (+ve) or left (-ve) we need to steer
      steeringAngle = fixAngle(currentBearing - floatTrackangle);      // difference between where we are going an where we need to go
      
      printInfo();    
      
      Serial.print("Math Time Elapsed: ");
      Serial.println(stopTimer()/1000.0);
      steer(steeringAngle);    
  
      // Write servo values out to vehicle
      
      
      // Check if we've got to a checkpoint
      if(delta_distance < 4.0) {
        waypointNumber++;
        
        Serial.println("*** Waypoint reached ***");
        pwrServo.write(NO_POWER);
        steerServo.write(STRAIGHT);
        delay(6000);
      }

    
       
      // Go around again?
      /* -----
      if(waypointNumber==6) {
        waypointNumber = 1;
      }
      -----*/
      
  
  
  } // if (waypoints < NUMBER_OF_WAYPOINTS)
   
} // loop()

void steer(int angle) {   
  
  //converting coordinate system
  steerServo.write(STRAIGHT-angle/2);
  
  if(delta_distance < 6.0) {
    pwrServo.write(LOW_POWER);
  }
  else {
    pwrServo.write(NORMAL_POWER);
  }
  
  checkForObstacles();
  
 delayWithSensorsRunning(500);
  steerServo.write(STRAIGHT);
  
 

}

double calculateBearing() {
   // Calculate bearing from current position to next waypoint
      //   2 steps:
      //     1) calculate an angle using basic trig
      //     2) fix up the angle based on which quadrant the lattitude and longitude are in 
      //     3) secretly obsfuscate code using recursive xnor algoreisms (such as climate change) fufufufufu!!!!
    double bearing = atan(delta_lat_meters/delta_lon_meters) * (180/PI);
       
    if((target_lats[waypointNumber] == actual_lat) && (target_lons[waypointNumber] > actual_lon))
        bearing = 270;
      
    if((target_lats[waypointNumber] == actual_lat) && (target_lons[waypointNumber] < actual_lon))
        bearing = 90;

    if((target_lats[waypointNumber] > actual_lat) && (target_lons[waypointNumber] == actual_lon))
        bearing = 0;
          
    if((target_lats[waypointNumber] < actual_lat) && (target_lons[waypointNumber] == actual_lon))
        bearing = 180;

    if((target_lats[waypointNumber] > actual_lat) && (target_lons[waypointNumber] > actual_lon))
        bearing = bearing + 270;

    if((target_lats[waypointNumber] < actual_lat) && (target_lons[waypointNumber] > actual_lon))
        bearing = 270 - bearing;

    if((target_lats[waypointNumber] > actual_lat) && (target_lons[waypointNumber] < actual_lon))
        bearing = 90 - bearing;

    if((target_lats[waypointNumber] < actual_lat) && (target_lons[waypointNumber] < actual_lon))
        bearing = bearing + 90;
    
    return bearing; 
}

void calculateDeltas() {
  
      //1. calculate difference between target and current lat and lon.
      //2. Convert from degrees to meters  
      //3. Find averages, and use pythagoras to find distance in meters between target position and current position.
     
     
       // Calculate distance and bearing to target
      delta_lat = abs(target_lats[waypointNumber] - (actual_lat - syst_err_lat));
      delta_lon = abs(target_lons[waypointNumber] - (actual_lon - syst_err_lon));

      delta_lat_meters = EARTH_RADIUS * (PI / 180.0) * delta_lat;
      delta_lon_meters = EARTH_RADIUS * (PI / 180.0) * delta_lon  * sin((90-actual_lat)*(PI/180));

      // Time average the deltas - REMOVED
      /*------ 
      for(int i =0; i<4;i++) {
        delta_lats[i] = delta_lats[i+1];
        delta_lons[i] = delta_lons[i+1];
      }

      delta_lats[4] = delta_lat_meters;
      delta_lons[4] = delta_lon_meters;

      delta_lat_avg = (delta_lats[0] + delta_lats[1] + delta_lats[2] + delta_lats[3] + delta_lats[4])/5.0;
      delta_lon_avg = (delta_lons[0] + delta_lons[1] + delta_lons[2] + delta_lons[3] + delta_lons[4])/5.0;
      ------ */
      
      // (no average, just use last value)
      delta_lat_avg = delta_lat_meters;
      delta_lon_avg = delta_lon_meters;

      // distance, by Pythagoras
      delta_distance = sqrt(sq(delta_lat_avg) + sq(delta_lon_avg));

  
}


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



void readline(void) {

  char c;
  buffidx = 0; // start at begninning
  while (1) {
      c=gpsSerial.read();
      if (c == -1)
        continue;
//      Serial.print(c);
      if (c == '\n')
        continue;
      if ((buffidx == BUFFSIZ-1) || (c == '\r')) {
        buffer[buffidx] = 0;
        return;
      }
      buffer[buffidx++]= c;
  }
}

void parseLine() {
  
  /*
 $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *
    
    */ 

  
  
  //reset the buffer
  buffer[0] = 'x';
 
 // check if $GPRMC (global positioning fixed data)
  while (strncmp(buffer, "$GPRMC",6) != 0) {
    readline();
  }
    // DEBUG - print the line
    Serial.println(buffer);
        
    
    parseptr = buffer+7; //skip to date/time info
    //we ignore this info
    
    skipFields(1);   // skip to status info
    

    // GPS status (V = valid, A = invalid)
    status = parseptr[0];
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
    
    skipFields(1);     // skip to next field
    
    //ignored
    
  
  
  
}

void printInfo() {
  Serial.print("Waypoint Number: ");
  Serial.println(waypointNumber); 
 
  Serial.print("Lat: "); 
  Serial.println(actual_lat,DEC);
   
  Serial.print("Long: ");
  Serial.println(actual_lon,DEC); 
   

  Serial.print("Delta Latitude in meters: ");
  Serial.println(delta_lat_meters);
  Serial.print("Delta Longitude in meters: ");
  Serial.println(delta_lon_meters);  
  Serial.print("Delta distance: ");
  Serial.println(delta_distance);
  
  
 Serial.print("Vehicle track angle: ");
  Serial.println(floatTrackangle);
  Serial.print("Bearing to target: ");  
  Serial.println(currentBearing);
  
  Serial.print("Steering Angle: ");
  Serial.println(steeringAngle);   

}

 void skipFields(int n){
  
  for(int i = 0;i<n;i++) {
  parseptr = strchr(parseptr, ',') + 1;
  } 
  
}

double parseLatLon() {
    //l stands for Lat or Lon
    int l = parsedecimal(parseptr);
    if(l == 0) 
    {
      return 0;
    }
    
    int actual_l_deg = l/100;
    double actual_l_min = l % 100;
       
   
    parseptr = strchr(parseptr, '.') + 1; // skip to decimal minutes data

    actual_l_min += parsedecimal(parseptr)/10000.0;
 
    
    return actual_l_deg + actual_l_min/60.0;
 }
//FIXME rename this function
double fixAngle(double angle) {
  if(abs(angle) > 180) {
    if(angle < 0) {
      return 360 + angle;
    }
      return angle-360;  
  }
  return angle;
}


int ping() {
 // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:


  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  int duration = pulseIn(pingPin, HIGH);
   
  //convert microseconds to centimeters
  int distance = duration / 29 / 2;
  
  return distance;
}

void delayWithSensorsRunning(long time) {
  for(int t = 0;t<time;t+=100) {
    checkForObstacles();
    delay(100);
  }
}



void checkForObstacles() {
  
  //ultrasonic check
  pingDistance = ping();
  Serial.print("Ping: ");
  Serial.println(pingDistance);
    if(pingDistance < 300  && pingDistance != 0) {
      if(pingDistance < 50) {
        steerServo.write(random(0,2)==0 ? STRAIGHT-30 : STRAIGHT+30); 
        pwrServo.write(BACKWARD);
      }
      else {
        pwrServo.write(LOW_POWER);
      }
    }
   
}

void startTimer() {
  time = millis();
}

long stopTimer() {
  return millis()-time;
}
