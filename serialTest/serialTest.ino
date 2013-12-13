#include <SoftwareSerial.h>
#include <PWMServo.h>

PWMServo aServo;
SoftwareSerial mySerial(10,11);

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
}
int d;
void loop()
{
  if(mySerial.available()){
    char c = mySerial.read();
    if(c) {
      d = c;
      Serial.println(d);
    }
  }
}
