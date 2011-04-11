#include <Wire.h>
#include "CrawlerHeader.h"

#define pingPin 7
#define touchPin 6

int distance;

void setup() {
  pinMode(touchPin, INPUT);
  
  Wire.begin(WIRE_ADDRESS_SENSORS);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  
}

void loop() {
  
  int pingDistance = ping();
  
  if(pingDistance==0) {
    pingDistance = 999;
  }
  
  int touchValue = digitalRead(touchPin);
  if(touchValue == HIGH) {
   distance = 1;
  }
  else {
  distance = pingDistance;
  }
  
  delay(100);
}


void requestEvent() {
  Wire.send(distance);
  
  Serial.print("Sent: ");
  Serial.println(distance);
}





int ping() {
  //UltraSonic Ping, returns distance in cm
  
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
