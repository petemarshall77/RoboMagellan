/*
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

#define rxPin 2
#define txPin 3

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
#include <Wire.h>

// Serial and servo ports
SoftwareSerial gpsSerial = SoftwareSerial(txPin,rxPin);

// Define course waypoints
//  Assumption: program will run in Northern Hemisphere and West of Prime Meridian - i.e. GPS indicators will be N and W - so we
//  don't include them in the course definition.
//  *** test track around PVHS blacktop outside room 501 ***


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
double target_lat= 0;
double target_lon = 0;

// Current delta from last position
 double delta_lat, delta_lon;                                              // in degrees

// Delta in position in meters
double delta_lat_meters, delta_lon_meters, delta_distance;

//GPS systematic error
boolean calibrated = false;
double syst_err_lat;
double syst_err_lon;


int steeringAngle;

//special variables for sending
double distance_send = 0;
int steeringAngle_send = 0;




// SETUP - the action starts here!
void setup() 
{ 
  // Set pin statuses as needed
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  // Start communication
  Serial.begin(GPSRATE);          // arduino --> serial port monitoring (via USB cable)
  gpsSerial.begin(GPSRATE);       // GPS shield <--> arduino

  Wire.begin(1);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent); 
 

  // Show we're alive 
  Serial.println("ROBOMAGELLAN: Version 4"); 
  
  //THIS NEEDS TO BE RE-WORKED
  //We need to get our first coordinates from the MASTER before we can
  //calculate syst-err
  
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
   syst_err_lat = lat_sum/10.0 - target_lat;
   syst_err_lon = lon_sum/10.0 - target_lon;
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
 
         
      
      //parseLine() returns 
      //actualLat, actualLon, floatTrackAngle, status, latDir, lonDir
      
      parseLine();
      
      //calculateDelta() returns
      //delta_lat,delta_lon
      //delta_lat_meters,delta_lon_meters
      //delta_lat_avg,delta_lon_avg,delta_distance      
      
      calculateDeltas();
     
     
      currentBearing = fixAngle(calculateBearing()); 
      // Calculate a steering angle - the degrees right (+ve) or left (-ve) we need to steer
      steeringAngle = fixAngle(currentBearing - floatTrackangle);      // difference between where we are going an where we need to go
      
      //prepare variables for sending
      distance_send = delta_distance;
      steeringAngle_send = steeringAngle;
      
      
      printInfo();    
          
} 


double calculateBearing() {
   // Calculate bearing from current position to next waypoint
      //   2 steps:
      //     1) calculate an angle using basic trig
      //     2) fix up the angle based on which quadrant the lattitude and longitude are in 
      //     3) secretly obsfuscate code using recursive xnor algoreisms (such as climate change) fufufufufu!!!!
    double bearing = atan(delta_lat_meters/delta_lon_meters) * (180/PI);
       
    if((target_lat == actual_lat) && (target_lon > actual_lon))
        bearing = 270;
      
    if((target_lat == actual_lat) && (target_lon < actual_lon))
        bearing = 90;

    if((target_lat > actual_lat) && (target_lon == actual_lon))
        bearing = 0;
          
    if((target_lat < actual_lat) && (target_lon == actual_lon))
        bearing = 180;

    if((target_lat > actual_lat) && (target_lon > actual_lon))
        bearing = bearing + 270;

    if((target_lat < actual_lat) && (target_lon > actual_lon))
        bearing = 270 - bearing;

    if((target_lat > actual_lat) && (target_lon < actual_lon))
        bearing = 90 - bearing;

    if((target_lat < actual_lat) && (target_lon < actual_lon))
        bearing = bearing + 90;
    
    return bearing; 
}

void calculateDeltas() {
  
      //1. calculate difference between target and current lat and lon.
      //2. Convert from degrees to meters  
      //3. Find averages, and use pythagoras to find distance in meters between target position and current position.
     
     
       // Calculate distance and bearing to target
      delta_lat = abs(target_lat - (actual_lat - syst_err_lat));
      delta_lon = abs(target_lon - (actual_lon - syst_err_lon));

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


void requestEvent() {
  //This is called when ever the MASTER does requestFrom() 
  //send distance, then steeringAngle
  
  //Codes:
  //999='No Signal',888='Need Co-ords to calc syst error',777='Need next waypoint'
  if(actual_lat == 0) {
    Wire.send(0);
    Wire.send(999);
  }
  else if(!calibrated) {
    Wire.send(0);
    Wire.send(888);
  }
  else if(target_lat == 0) {
    //The plan is to reset target_lat and target_lon to zero whenever you want a new coordinate
    Wire.send(0);
    Wire.send(777);
  }
  else {
  //is int precision good enough for delta_distance (Please say yes!)
  Wire.send((int)delta_distance_send);
  Wire.send(steeringAngle_send);
  }
  Serial.print("Sent::");
  Serial.print("Dist: ");
  Serial.print(delta_distance_send);
  Serial.print("; Steering Angle: ");
  Serial.println(steeringAngle_send);
}

void receiveEvent(int bytes) {
    //i THINK this is called whenever the MASTER sends us anything
    
    //get target_lat or target_lon from the MASTER  
    
    //I'm going under an assumption that might be totally WRONG
    /*
    The assumption is that receiving the target_lat will trigger
    one receiveEvent(), and receiving the target_lon will
    subsequently trigger another.
    */
   
    char buf[bytes];
    int i = 0;
    while(Wire.available()) {
      char c = (char)Wire.receive();
      buf[i]= c;
      i++;
      }
      
       //I think this strategy is too risky in case of a communication issue!
    if(target_lat == 0 && target_lon == 0) {
     target_lat = atof(buf);
    }
    else if(target_lon == 0) {
     target_lon = atof(buf);
    }
      
  
     

}


