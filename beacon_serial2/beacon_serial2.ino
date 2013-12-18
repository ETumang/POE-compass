#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial GPSSerial(9,10); //TX, RX

Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();
 
//int greenPin = 6;
//int bluePin = 7;

void setup() {

  Serial.begin(9600);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
 
//  pinMode(redPin, OUTPUT);
//  pinMode(greenPin, OUTPUT);
//  pinMode(bluePin, OUTPUT);

}

//void setColor(int red, int green, int blue)
//{
//  analogWrite(greenPin, green);
//  analogWrite(bluePin, blue);
//  analogWrite(redPin, red);
//}

void loop() { 
//  if (timer < 300000) {
//    setColor(0, 255, 0);} // green
//  if ((timer > 300000) && (timer < 600000)) {
//    setColor(0, 0, 255);} // blue
//  if ((timer > 600000) && (timer < 900000)) {
//    setColor(255, 0, 0);} // red
//  if (timer > 900000) {
//    digitalWrite(greenPin, LOW); }// turn on pullup resistors

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return; // If we miss something then just get it next loop
  }

  if (millis() - timer > 1999) { // Run once every 2 seconds
    if(GPS.fixquality){ // For some reason gps.fix is returning 0 even when we get a satellite
      long lat = GPS.latitude * 10000; // Convert things to longs
      long lon = GPS.longitude * 10000;
      char latChar[8];
      char lonChar[9];
      dtostrf(lat,8,0,latChar);
      if(GPS.lat == 'S') // Account for hemisphere
        Serial.write('-');
      if(GPS.lat == 'N')
        Serial.write('+');
      Serial.write(latChar); // need to write this before running dtostrf again
      dtostrf(lon,9,0,lonChar);
      if (lon < 100000000) {
        lonChar[0] = '0'; // For longitudes < 100 to preserve the digit
      }
      if(GPS.lon == 'W') // Hemisphere again
        Serial.write('-');
      if(GPS.lon == 'E')
        Serial.write('+');
      Serial.write(lonChar);
      // Ends up writing 19-length char array of (sign, eight digits, sign, nine digits)
    }
  if(!GPS.fixquality){
    Serial.println("Waiting for a fix...");
  }
    timer = millis();
  }
}
