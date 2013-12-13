// Libraries
#include <SPI.h>
#include <PWMServo.h>
#include <Wire.h>
#include <LSM303.h>
//#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>
//#include <RF22Mesh.h>
//#include <RF22.h>
//#include <RF22Router.h>
//#include <RF22Datagram.h>

// Start sensors
LSM303 compass;
PWMServo aServo;

//SoftwareSerial mySerial(8, 4); // For GPS
//Adafruit_GPS GPS(&mySerial);
//#define GPSECHO  false;
//boolean usingInterrupt = false;
//void useInterrupt(boolean);

//RF22 rx;
//hold raw RF data
//uint8_t buf[10];
//uint8_t len = sizeof(buf);
int rfTime;
int last_rx;
boolean haveRf = false; // Have we ever received a transmission
float beaconPos[2]; //longitude and latitude of beacons

//led 
unsigned long time; //integers in the range 0 to 4,294,967,295
//Number of milliseconds since the program started (unsigned long) 
int greenPin = 5;
int bluePin = 6;
int redPin = 7;

void setup() {
  Serial.begin(9600);
  aServo.attach(9);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-643, -606, -577}; // Compass calibration
  compass.m_max = (LSM303::vector<int16_t>){+647, +555, +531};
  
//  GPS.begin(9600);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Maybe RMC only?
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PGCMD_ANTENNA);
 
//  rx.init();
//  rx.setModeRx();
    
  rfTime = 0;
  last_rx = 0;
  
  //led 
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
 
}

// Constants
uint32_t timer = millis();
float pi = 3.141592653;

//led
void setColor(int red, int green, int blue)
{
analogWrite(greenPin, green);
analogWrite(bluePin, blue);
analogWrite(redPin, red);

}

void loop() {
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

  // Constantly reads for new GPS data
//  char c = GPS.read();
//  if (GPS.newNMEAreceived()) {
//    if (!GPS.parse(GPS.lastNMEA()))
//      return; // If we miss something then just get it next loop
//  }
  rfTime = millis() - last_rx; // Time since last RF will increase until it gets new transmission
  if (Serial.available() >= 8)   // Wait for two longs worth of data
    getRf(); 

  if (timer > millis())
    timer = millis(); // In case one of these rolls over max value
  if (millis() - timer > 2000) { // Run once/2 seconds
  
//    if (GPS.fix && haveRf && (millis() - rfTime < 10000)) {
    if (haveRf && (millis() - rfTime < 10000)) {
      // read from magnetometer
      float current = getHeading();
  
      // TESTING
//      float rawlat1 = GPS.latitude;
//      float rawlong1 = GPS.longitude;
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); //Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); //Serial.println(GPS.lon);
      
      // Example Needham-ish Coordinates
      float hyplat1 = 0100.0000;
      float hyplong1 = 0000.0000;
//      float hyplat2 = 5700.6081;
//      float hyplong2 = -7155.8617;
//      float target = calcTarget(rawlat1, rawlong1, beaconPos[0], beaconPos[1]);
      float target = calcTarget(hyplat1, hyplong1, beaconPos[0], beaconPos[1]);
      //Serial.print("Target Heading: ");
      //Serial.println(target);
      //Serial.print("Current Heading: ");
      //Serial.println(current);
      //Serial.print("Move motor to: ");
      setServo(target, current);
    }
    
//    if (!(GPS.fix))
      //Serial.println("Working on getting a fix...");
//    if (!(haveRf))
      //Serial.println("Waiting for a transmission...");
//    if ((millis() - rfTime > 10000))
      //Serial.println("Waiting for fresh RF data...");
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
  //Serial.print("Change in Latitude: ");
  //Serial.println(dlat);
  //Serial.print("Change in Longitide: ");
  //Serial.println(dlong);
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
  //Serial.println(motorpos);
  aServo.write(motorpos);
}

void getRf(){
//    if (rx.recv(buf,&len)){
    long raw[2];
    Serial.readBytes((char*) &raw,8); // Read over serial. Automatically divides into array.
    beaconPos[0] = float(raw[0])/10000;
    beaconPos[1] = float(raw[1])/10000;
    haveRf = true;
//    rx.setModeRx();
    last_rx = millis();
//    }
}
  
