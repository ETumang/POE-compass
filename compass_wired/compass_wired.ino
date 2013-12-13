// Libraries
#include <SPI.h>
#include <PWMServo.h>
#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>

LSM303 compass;
PWMServo aServo;

SoftwareSerial mySerial(8, 4);

int rfTime;
int last_rx;
boolean haveRf = false; // Have we ever received a transmission
float beaconPos[2]; //longitude and latitude of beacons

void setup() {
  Serial.begin(9600);
  Serial.println("Hello world");
  delay(1000);
  Wire.begin();
  aServo.attach(9);
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-643, -606, -577}; // Compass calibration
  compass.m_max = (LSM303::vector<int16_t>){+647, +555, +531};
  rfTime = 0;
  last_rx = 0;
  mySerial.begin(9600);
  Serial.println("Hello again");
}

// Constants
uint32_t timer = millis();
float pi = 3.141592653;

void loop() {
  rfTime = millis() - last_rx; // Time since last RF will increase until it gets new transmission
  if (mySerial.available() >= 8)   // Wait for two longs worth of data
    getRf(); 

  if (timer > millis())
    timer = millis(); // In case one of these rolls over max value
  if (millis() - timer > 2000) { // Run once/2 seconds
    if (haveRf && (millis() - rfTime < 10000)) {
      // read from magnetometer
      float current = getHeading();
      float hyplat1 = 0100.0000;
      float hyplong1 = 0000.0000;
      float target = calcTarget(hyplat1, hyplong1, beaconPos[0], beaconPos[1]);
      Serial.print("Target Heading: ");
      Serial.println(target);
      Serial.print("Current Heading: ");
      Serial.println(current);
      Serial.print("Move motor to: ");
      setServo(target, current);
    }
    if (!(haveRf))
      Serial.println("Waiting for a transmission...");
      Serial.println(mySerial.available());
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
  float target = (atan2(dlong,dlat) * 180 / pi) - 90; // Accounts for the orientation of the magnetometer in the box
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
    long raw[2];
    mySerial.readBytes((char*) &raw,8); // Read over serial. Automatically divides into array.
    beaconPos[0] = float(raw[0])/10000;
    beaconPos[1] = float(raw[1])/10000;
    haveRf = true;
    last_rx = millis();
}
  
