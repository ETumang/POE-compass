#include <Libraries.h>

Mag mag = Mag(uniqueID); // initialize magnetometer

int in1Pin = 12;
int in2Pin = 11;
int in3Pin = 10;
int in4Pin = 9;
Stepper stepper(720, in1Pin, in2Pin, in3Pin, in4Pin);  

const float Pi = 3.1416;
float targetHeading = 0.0;
const int RFreq = 800;
int updateInterval = 1000;

void setup() {
  RF.initialize();
  GPS.initialize();
  
  Serial.begin(9600);
  analogReference (EXTERNAL);
  int timeOffset = RF.receive(initTime);
  
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  
  stepper.setSpeed(10);
}

void loop() {
  magX = mag.getX();
  magY = mag.getY();
  float currentHeading = (atan2(magY,magX) * 180) / Pi;
  if (currentHeading < 0) {
    currentHeading = currentHeading + 360;
  }
  
  float latitude = GPS.getLat(); // Y
  float longitude = GPS.getLong(); // X
  if (longEAST) {
    longitude = 360 - longitude;
  }
  float beaconGPS[5] = RF.receive(RFreq);
  float beaconLat = beaconGPS[1];
  float beaconLong = beaconGPS[2];
  if (beaconEAST) {
    beaconLong = 360 - beaconLong;
  }
  float deltaY = beaconLat - latitude;
  float deltaX = beaconLong - longitude;
  
  targetHeading = (atan2(deltaY,deltaX) * 180) / Pi;
  if (targetHeading < 0) {
    targetHeading = targetHeading + 360;
  }
  float error = currentHeading - targetHeading;
  if(error > 180) {
    error = error - 360;
  }
  
  stepper.step(error * 720 / 360 / 2);
  delay(updateInterval);
}

// set up magnetometer
// set up time offset
// set up motor
// initialize other instruments
// loop magnetometer readings
// loop convert magnetometer to heading
// loop receive beacon signal (GPS coordinates)
// loop convert self + beacon GPS into vector heading
// loop compare current heading to target heading
// loop correct motor for heading
// Try to get rid of floating points later
