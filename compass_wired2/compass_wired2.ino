// Libraries
#include <SPI.h>
#include <PWMServo.h>
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Start sensors
LSM303 compass;
PWMServo aServo;
SoftwareSerial GPSSerial(6,5); //TX, RX
Adafruit_GPS GPS(&GPSSerial);

// Time variables
int rfTime;
int last_rx;
boolean haveRf = false; // Have we ever received a transmission
float beaconPos[2]; //longitude and latitude of beacons

// Math Constants
uint32_t timer = millis();
float pi = 3.141592653;
long exponents[9] = {1,10,100,1000,10000,100000,1000000,10000000,100000000};

void setup() {
  Serial.begin(9600);
  aServo.attach(9);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-643, -606, -577}; // Compass calibration
  compass.m_max = (LSM303::vector<int16_t>){+647, +555, +531};
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Maybe RMC only?
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
    
  rfTime = 0;
  last_rx = 0;
}

void loop() {

  //Constantly reads for new GPS data
 char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return; // If we miss something then just get it next loop
  }
  rfTime = millis() - last_rx; // Time since last RF will increase until it gets new transmission
  if(Serial.available() == 19){ // Looking for the right length char array
    getRf();
  }
  if (timer > millis())
    timer = millis(); // In case one of these rolls over max value
  if ((millis() - timer > 1999)) { // Run once per 2 seconds
    Serial.print("Time since last: ");
    Serial.println(rfTime);
    if (GPS.fixquality && haveRf && rfTime < 10000) {
    //if(haveRf && rfTime < 10000) {
      // read from magnetometer
      float current = getHeading();
  
      // TESTING
      //float rawlat1 = 4217.6081;
      //float rawlong1 = -07115.8617;
      float rawlat1 = GPS.latitude;
      float rawlong1 = GPS.longitude;
      float target = calcTarget(rawlat1, rawlong1, beaconPos[0], beaconPos[1]);
      Serial.print("Target Heading: ");
      Serial.println(target);
      Serial.print("Current Heading: ");
      Serial.println(current);
      Serial.print("Move motor to: ");
      setServo(target, current);
    }
    if (!(GPS.fix))
      Serial.println("Waiting on a fix");
    if (!(haveRf))
      Serial.println("Waiting for a transmission...");
    if (rfTime > 10000)
      Serial.println("Waiting for fresh RF data...");
    timer = millis(); // Update Timer 
  }
}

float getHeading() {
  compass.read();
  float heading = compass.heading((LSM303::vector<int>){1, 0, 0}) - 14.49; // Referenced to +X-axis, corrected for declination
  return heading;
}

float calcTarget(float rawlat1, float rawlong1, float rawlat2, float rawlong2){
  float lat1 = convertDeg(rawlat1);
  float long1 = convertDeg(rawlong1);
  float lat2 = convertDeg(rawlat2);
  float long2 = convertDeg(rawlong2);
  float dlat = (lat2 - lat1);
  float dlong = (long2 - long1) * cos((lat2 + lat1) / 360 * pi); // Longitude degrees narrow as you approach the poles
  Serial.print("Change in Latitude: ");
  Serial.println(dlat);
  Serial.print("Change in Longitide: ");
  Serial.println(dlong);
  float target = (atan2(dlong,dlat) * 180 / pi) - 90; // -90 accounts for the orientation of the magnetometer in the box
  if (target < 0) {
    target = target + 360;
  }
  return target;
}

// Converts ddmm.mmmmm
float convertDeg(float reading){
  long raw = reading * 10000;
  float degrees = raw / 1000000; // Undo the 10000x and an additional 100x
  Serial.print("Degrees: "); Serial.println(degrees);
  float minutes = (raw % 1000000);
  minutes = minutes / 600000; // Undo the 10000x, 60 minutes per degree 
  degrees = (degrees) + minutes; // Result is decimal degrees
  return degrees;
}

void setServo(float target, float current) { // Motor is geared at 3.5 degrees / degree
  int motorpos = ceil((target - current) / 3.5); // We want to convert 0-360 into 0-102
  if (motorpos > 102)
    motorpos = 102; // Sanity check
  if (motorpos < 0)
    motorpos = 102 + motorpos; // Account for negatives
  motorpos = ceil(motorpos * 0.94); // Further convert from 0-102 to 0-96 due to servo error.
  Serial.println(motorpos);
  motorpos = motorpos + 42; // Place the position in the middle of the servo range for less error.
  Serial.print("Motor position: ");
  
  aServo.write(motorpos);
}

void getRf(){
    //Assumes 19 chars, 1 sign, 8 latitude, 1 sign, and 9 longitude
    char rawLatArr[9];
    char rawLongArr[10];
    Serial.readBytes(rawLatArr,9); // Example: +XXXXXXXX
    Serial.readBytes(rawLongArr,10); // Example: +XXXXXXXXX
    long rawToLat = 0;
    long rawToLong = 0;
    for (int i = 0; i < 8; i++) {
      long j = (rawLatArr[8-i] - '0') * exponents[i]; // positions 1-8 out of 0-8
      long k = (rawLongArr[9-i] - '0') * exponents[i]; // positions 2-9 out of 0-9
      rawToLat = rawToLat + j;
      rawToLong = rawToLong + k;
    }
    
    int thingy = rawLongArr[1] - '0'; // Handle last longitude digit
    rawToLong = rawToLong + (exponents[8] * thingy); // position 1 out of 0-9 (longitude)
    
    if (rawLatArr[0] == '-'){
      rawToLat = rawToLat * -1;
    }
    if (rawLongArr[0] == '-'){
      rawToLong = rawToLong * -1;
    }
    Serial.println(rawToLat);
    Serial.println(rawToLong);
    beaconPos[0] = float(rawToLat)/10000;
    beaconPos[1] = float(rawToLong)/10000;
    haveRf = true;
    last_rx = millis();
}
  
