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

int RFarray[3]; // size of array

uint8_t data[3];

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
  
  Serial.print("Hello");  
  if (beacon.init()){
    Serial.print("ready");
  }
  

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

void loop()

{

GPS.read();

if (timer > millis()) timer = millis();

// approximately every 2 seconds or so, print out the current stats

if (millis() - timer > 2000) {

  timer = millis(); // reset the timer 

if (buttonpushed == false)

{

  total_time = millis();

}

diff_time = total_time- last_time;

if ( (GPS.fix)& (buttonpushed == true) or (GPS.fix)&  (diff_time > 60000)){ //ten seconds

  //because of the 2 second timer, the button_time will be actually half
 
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

  RFarray[2] = GPS.speed*10000;
  
  uint8_t data[] = {RFarray[0], RFarray[1]};

  beacon.send(data,sizeof(data));
  Serial.print(data, DEC);  

  beacon.waitPacketSent();
  
  uint8_t buf[RF22_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (beacon.waitAvailableTimeout(2000))
    { 
      // Should be a message for us now   
      if (beacon.recv(buf, &len))
      {
        Serial.print("Hi");
      }

    }

  Serial.print("RF latitude ");

  /*Serial.print (RFarray[0]);

  Serial.print ("RF longitude ");

  Serial.println (RFarray[1]);

  Serial.print ("GPS speed ");

  Serial.println (RFarray[2]);

  Serial.println ( total_time);

  Serial.println("     time for button ");

  Serial.println(total_time - last_time);*/
}

}


