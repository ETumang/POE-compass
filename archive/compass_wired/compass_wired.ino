// Libraries
#include <SPI.h>
#include <PWMServo.h>
#include <Wire.h>
#include <LSM303.h>
<<<<<<< HEAD
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
//#include <RF22Mesh.h>
//#include <RF22.h>
//#include <RF22Router.h>
//#include <RF22Datagram.h>
=======
#include <SoftwareSerial.h>
>>>>>>> 76bd36fe90045b6f6031fdd5fcf0884fd41aa10e

LSM303 compass;
PWMServo aServo;

<<<<<<< HEAD
SoftwareSerial mySerial(8, 4); // For GPS
SoftwareSerial GPSSerial(5,6);
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO  false;
boolean usingInterrupt = false;
void useInterrupt(boolean);
=======
SoftwareSerial mySerial(8, 4);
>>>>>>> 76bd36fe90045b6f6031fdd5fcf0884fd41aa10e

int rfTime;
int last_rx;
boolean haveRf = false; // Have we ever received a transmission
float beaconPos[2]; //longitude and latitude of beacons

void setup() {
  Serial.begin(9600);
<<<<<<< HEAD
  mySerial.begin(9600);
  aServo.attach(9);
=======
  Serial.println("Hello world");
  delay(1000);
>>>>>>> 76bd36fe90045b6f6031fdd5fcf0884fd41aa10e
  Wire.begin();
  aServo.attach(9);
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-643, -606, -577}; // Compass calibration
  compass.m_max = (LSM303::vector<int16_t>){+647, +555, +531};
<<<<<<< HEAD
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Maybe RMC only?
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
 
  //rx.init();
  //rx.setModeRx();
    
=======
>>>>>>> 76bd36fe90045b6f6031fdd5fcf0884fd41aa10e
  rfTime = 0;
  last_rx = 0;
  mySerial.begin(9600);
  Serial.println("Hello again");
}

// Constants
uint32_t timer = millis();
float pi = 3.141592653;

void loop() {
<<<<<<< HEAD
  //led
    time = millis()/1000;// for now use minutes
  //prints time since program started
// Change time constants when you know the game length
if (time > 0 & time < 120) {
setColor(0, 0, 255);} // green
if (time >120 & time < 240) {
setColor(255, 0, 0);} // blue
if (time >240 & time <360 ) {
setColor(0, 255, 0);} // red
if (time >360 ) {
digitalWrite(greenPin, LOW); }// turn on pullup resistors

  //Constantly reads for new GPS data
 char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return; // If we miss something then just get it next loop
    }
  rfTime = millis() - last_rx; // Time since last RF will increase until it gets new transmission
  if (mySerial.available()>=8)   // Wait for two longs worth of data
=======
  rfTime = millis() - last_rx; // Time since last RF will increase until it gets new transmission
  if (mySerial.available() >= 8)   // Wait for two longs worth of data
>>>>>>> 76bd36fe90045b6f6031fdd5fcf0884fd41aa10e
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
<<<<<<< HEAD
    mySerial.flush();
=======
>>>>>>> 76bd36fe90045b6f6031fdd5fcf0884fd41aa10e
    last_rx = millis();
}
  
