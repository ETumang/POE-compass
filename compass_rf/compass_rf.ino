// Libraries
#include <SPI.h>
#include <PWMServo.h>
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <RF22Mesh.h>
#include <RF22.h>
#include <RF22Router.h>
#include <RF22Datagram.h>

// Start sensors
LSM303 compass;
PWMServo aServo;

SoftwareSerial mySerial(8, 4); // For GPS
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false;
boolean usingInterrupt = false;
void useInterrupt(boolean);

RF22 rx;
//hold raw RF data
uint8_t buf[10];
uint8_t len = sizeof(buf);
int rfTime;
int last_rx;
boolean haveRf = false; // Have we ever received a transmission
float beaconPos[2]; //longitude and latitude of beacons

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
 
  rx.init();
  rx.setModeRx();
    
  rfTime = 0;
  last_rx = 0;
}

// Constants
uint32_t timer = millis();
float pi = 3.141592653;

void loop() {

  // Constantly reads for new GPS data
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return; // If we miss something then just get it next loop
  }
  rfTime = millis() - last_rx;
  if (rx.available())   
    getRf(); 

  if (timer > millis())
    timer = millis(); // In case one of these rolls over max value
  if (millis() - timer > 2000) { // Run once/2 seconds
  
    if (GPS.fix && haveRf && (millis() - rfTime < 10000)) {
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
//      float hyplat2 = 5700.6081;
//      float hyplong2 = -7155.8617;
      float target = calcTarget(rawlat1, rawlong1, beaconPos[0], beaconPos[1]);
      Serial.print("Target Heading: ");
      Serial.println(target);
      Serial.print("Current Heading: ");
      Serial.println(current);
      Serial.print("Move motor to: ");
      setServo(target, current);
    }
    
    if (!(GPS.fix))
      Serial.println("Working on getting a fix...");
    if (!(haveRf))
      Serial.println("Waiting for a transmission...");
    if ((millis() - rfTime > 10000))
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
  float dlong = (long2 - long1) * cos((lat2 + lat1) / 360 * pi);
  Serial.print("Change in Latitude: ");
  Serial.println(dlat);
  Serial.print("Change in Longitide: ");
  Serial.println(dlong);
  float target = atan2(dlong,dlat) * 180 / pi;
  if (target < 0) {
    target = target + 360;
  }
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
  aServo.write(motorpos);
}

void getRf(){
    if (rx.recv(buf,&len)){
    long int lat = buf[0]|(buf[1]<<8)|buf[2]<<16|buf[3]<<24;
    long int lon = buf[4]|buf[5]<<8|buf[6]<<16|buf[7]<<24;
    beaconPos[0] = float(lat)/10000;
    beaconPos[1] = float(lon)/10000;
    haveRf = true;
    rx.setModeRx();
    last_rx = millis();
    }
}
  
