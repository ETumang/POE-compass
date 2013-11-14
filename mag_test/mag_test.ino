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
  Serial.println("Hi2");
  sensors_event_t event;
  
  accel.getEvent(&event);
  float Pi = 3.14159;
  // Calculate tilts
  float xTilt = 90 - (atan2(event.acceleration.z,event.acceleration.x) * 180)/ Pi;
  float yTilt = 90 - (atan2(event.acceleration.z,event.acceleration.y) * 180)/ Pi;
  float xAccel = event.acceleration.x;
  float yAccel = event.acceleration.y;
  float zAccel = event.acceleration.z;
  
  mag.getEvent(&event);
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  // Adjust for magnetic declination in area around Need
  heading = heading - 14.49;
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.println("Acceleration: ");
  Serial.print("X: "); Serial.print(xAccel); Serial.print("  ");
  Serial.print("Y: "); Serial.print(yAccel); Serial.print("  ");
  Serial.print("Z: "); Serial.print(zAccel); Serial.print("  ");Serial.println("m/s^2 ");
  Serial.print("X Tilt: "); Serial.println(xTilt);
  Serial.print("Y Tilt: "); Serial.println(yTilt);
  Serial.println("Magnetic Fields");
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("microTeslas ");
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  Serial.println();
  delay(1000);
  Serial.println("hi");
}
