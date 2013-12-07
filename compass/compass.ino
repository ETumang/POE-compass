// Libraries
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Start sensors
LSM303 compass;
Servo aServo;
SoftwareSerial mySerial(3, 2); // For GPS
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false;
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup() {
  Serial.begin(115200);
  Serial.println("Hi, this is working.");
  aServo.attach(4);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-643, -606, -577}; // Compass calibration
  compass.m_max = (LSM303::vector<int16_t>){+647, +555, +531};
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Maybe RMC only?
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}

// Constants
uint32_t timer = millis();
float pi = 3.141592653;

void loop() {
  
  // Constantly reads for new GPS data
  char c = GPS.read();
  //  if (GPSECHO)
  //      if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return; // If we miss something then just get it next loop
  }

  if (timer > millis())  timer = millis(); // In case one of these rolls over max value
  if (millis() - timer > 2000) { // Run once/2 seconds
  
    if (GPS.fix) {
      // read from magnetometer
      float current = getHeading();
  
      // TESTING
      float rawlat1 = GPS.latitude;
      float rawlong1 = GPS.longitude;
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      // Example Needham-ish Coordinates
//      float hyplat1 = 4217.6081;
//      float hyplong1 = -7115.8617;
      float hyplat2 = 5700.6081;
      float hyplong2 = -7155.8617;
      float target = calcTarget(rawlat1, rawlong1, hyplat2, hyplong2);
      Serial.print("Target Heading: ");
      Serial.println(target);
      Serial.print("Current Heading: ");
      Serial.println(current);
      Serial.print("Move motor to: ");
      setServo(target, current);
    }
    
    if (!(GPS.fix)) {
      Serial.println("Working on it...");
    }
    
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
  float dlong = (long2 - long1) * cos((lat2 + lat1) / 360 * pi);
  Serial.print("Change in Latitude: ");
  Serial.println(dlat);
  Serial.print("Change in Longitide: ");
  Serial.println(dlong);
  float target = atan2(dlong,dlat) * 180 / pi;
  return target;
}

float convertDeg(float reading){
  long raw = reading * 10000;
  float degrees = raw / 1000000;
  float minutes = (raw % 1000000);
  minutes = minutes / 600000;
  degrees = (degrees) + minutes;
  return degrees;
}

void setServo(float target, float current) {
  int motorpos = ceil((target - current) / 3.5);
  if (motorpos < 0) {
    motorpos = 102 + motorpos;
  }
  Serial.println(motorpos);
//  aServo.write(motorpos);
}
