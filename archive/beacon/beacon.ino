#include <SPI.h>
#include <RF22Mesh.h>
#include <RF22ReliableDatagram.h>
#include <RF22.h>
#include <RF22Router.h>
#include <RF22Datagram.h>
#include <GenericSPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

RF22 beacon;

SoftwareSerial mySerial(8,4);

Adafruit_GPS GPS(&mySerial);

long int RFarray[2]; // size of array

int buttonPin = 3;


int buttonState = digitalRead(buttonPin);

unsigned long last_time = 0;

long randNumber1; //integers in the range –2,147,483,648 to 2,147,483,647

long randNumber2;

long randNumber3;//#define GPSECHO true

boolean buttonpushed = true;

unsigned long total_time = 0;

unsigned long diff_time;

float timer = millis();
uint8_t data[64];

unsigned long time; //integers in the range 0 to 4,294,967,295
//Number of milliseconds since the program started (unsigned long) 
int greenPin = 5;
int bluePin = 6;
int redPin = 7;

void setup() {

  Serial.begin(9600);

  pinMode (buttonPin, INPUT);

  attachInterrupt(0,button, RISING);//RISING = The input state changes from LOW to HIGH

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // 1 Hz update rate

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Request updates on antenna status, comment out to keep quiet

  GPS.sendCommand(PGCMD_ANTENNA);
  
  
 
//Number of milliseconds since the program started (unsigned long) 
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

}

void button () {

if (buttonpushed){

  unsigned long now = millis();

  if (abs (now - last_time) >200){ 

  // a random number from -300 to 300

  randNumber1 = random(300,600);

  randNumber2 = random(300,600);

  randNumber3 = random(300,600);

 

  RFarray[0] += randNumber1;

  RFarray[1] += randNumber2;

  RFarray[2] += randNumber3;

  Serial.println ("button pushed");

  buttonpushed = false;

}

   last_time = now;

  //x = 100;

}

}
void setColor(int red, int green, int blue)
{
analogWrite(greenPin, green);
analogWrite(bluePin, blue);
analogWrite(redPin, red);

}

void loop()

{ 

  time = millis()*1000;// for now use minutes
  //prints time since program started

if (time > 0 & time < 5) {
setColor(0, 0, 255);} // green
if (time >5 & time < 10) {
setColor(255, 255, 0); }// yellow
if (time >10 & time <15 ) {
setColor(0, 255, 0);} // red
if (time >15 ) {
digitalWrite(greenPin, LOW); }// turn on pullup resistors
  
 GPS.read();

if (timer > millis()) timer = millis();

// approximately every 2 seconds or so, print out the current stats

if (millis() - timer > 2000) 

  timer = millis(); // reset the timer 

if (buttonpushed == false)

{

  total_time = millis();

}

diff_time = total_time- last_time;

if ( (GPS.fix)& (buttonpushed == true) or (GPS.fix)&  (diff_time > 60000)){ //one minute
 
  if (GPS.newNMEAreceived()) {   

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false

      return;  // we can fail to parse a sentence in which case we should just wait for another

}
}

 

// if millis() or timer wraps around, we'll just reset it

 

  // 3 decimal points at the moment

// Serial.print(GPS.latitude,3);

  //Serial.print(",");

  //Serial.println(GPS.longitude,3);

  //Serial.print("   ");
  
 RFarray[0] = GPS.latitude*10000;//gives out the latitude

  RFarray[1] = GPS.longitude*10000;// gives out longitude
  
  uint32_t lat= RFarray[0];
  uint32_t lon = RFarray[1]; 
 
  uint8_t la0 = lat;
  uint8_t la1 = lat>>8;
  uint8_t la2 = lat>>16;
  uint8_t la3 = lat>>24;
 
  uint8_t lo0 = lon;
  uint8_t lo1 = lon>>8;
  uint8_t lo2 = lon>>16;
  uint8_t lo3 = lon>>24;
 
  uint8_t data[] = {la0,la1,la2,la3,lo0,lo1,lo2,lo3};
  
  Serial.write(data,8);
  
  /*beacon.send(data,sizeof(data));  

  beacon.waitPacketSent();
  
  uint8_t buf[RF22_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

if (beacon.waitAvailableTimeout(10))
    { 
      // Should be a message for us now   
      if (beacon.recv(buf, &len))
      {
        Serial.print("Hi");
      }

    }*/

  /*Serial.print("RF latitude "); */

  /*Serial.print (RFarray[0]);

  Serial.print ("RF longitude ");

  Serial.println (RFarray[1]);

  Serial.print ("GPS speed ");

  Serial.println (RFarray[2]);

  Serial.println ( total_time);

  Serial.println("     time for button ");

  Serial.println(total_time - last_time); */
 
}






