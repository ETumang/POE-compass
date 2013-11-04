#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

Adafruit_LSM303_Mag mag = Adafruit_LSM303_Mag(12345);
Adafruit_LSM303_Accel accel = Adafruit_LSM303_Accel(54321);

void setup()  
{
  Serial.begin(9600);
  Serial.println("Adafruit accelerometer/magnetometer library basic test!");
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void loop()
{
  Serial.print("Hi.");
  sensors_event_t event;
  Serial.print("Hi the second.");
  // There's a problem here
  mag.getEvent(&event);
  Serial.print("Hi again.");
  accel.getEvent(&event);
  Serial.print("Hi2.");
  float Pi = 3.14159;
  Serial.print("Hi3.");
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  Serial.print("Hi4.");
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.print("Hi5.");
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  delay(100);
}
