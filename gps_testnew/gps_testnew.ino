#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
float RFarray[2]; // size of array

//#define GPSECHO true
void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
}
float timer = millis();
void loop() {
GPS.read();
if (timer > millis()) timer = millis();
// approximately every 2 seconds or so, print out the current stats
if (millis() - timer > 2000) {
  timer = millis(); // reset the timer

  
RFarray[0] = GPS.latitude;//gives out the latitude
RFarray[1] = GPS.longitude;// gives out longitude

  if (GPS.newNMEAreceived()) {
    
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
// if millis() or timer wraps around, we'll just reset it


 
  // 3 decimal points at the moment
 // Serial.print(GPS.latitude,3);
  //Serial.print(",");
  //Serial.println(GPS.longitude,3);
  //Serial.print("   ");
  Serial.print("RF latitude");
  Serial.print (RFarray[0]);
  Serial.print ("RF longitude");
  Serial.println (RFarray[1]);
  //Serial.println(GPS.fix,3);
  //Serial.println(GPS.day,3); //Serial.print('/');
  //Serial.print(GPS.month, DEC); //Serial.print("/20");
//  Serial.println(GPS.year, DEC);
//  Serial.print (GPS.day,DEC);

}
}
