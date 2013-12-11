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
  sensors_event_t mag_event;
  sensors_event_t accel_event;
  
  accel.getEvent(&accel_event);
  float Pi = 3.14159;
  float xAccel = accel_event.acceleration.x;
  float yAccel = accel_event.acceleration.y;
  float zAccel = accel_event.acceleration.z;
  // Calculate tilts
  float xTilt = 90 - (atan2(zAccel,xAccel) * 180)/ Pi;
  float yTilt = 90 - (atan2(zAccel,yAccel) * 180)/ Pi;

  mag.getEvent(&mag_event);
  float xMag = mag_event.magnetic.x;
  float yMag = mag_event.magnetic.y; 
  float zMag = mag_event.magnetic.z;
  // Calculate the angle of the vector y,x
//  float heading = ((atan2(yMag,xMag) * 180) / Pi) - 14.8164;  // Adjust for magnetic declination in area around Needham
  float heading = ((atan2(yMag,xMag) * 180) / Pi) - 12.3167;  // Adjust for magnetic declination in area around Jamison
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
//  
//  float xMagTC = (xMag * cos(xTilt) + zMag * sin(xTilt)) - 48.402 * cos(xTilt); // cancel out the zaxis
//  float yMagTC = (xMag * sin(yTilt) * sin(xTilt) + yMag * cos(yTilt) - zMag * sin(yTilt) * cos(xTilt)) - 48.402 * cos(yTilt); // cancel out the zaxis
////  float headingTC = ((atan2(yMagTC,xMagTC) * 180) / Pi) - 14.8164; //adjust for magnetic declination
//  if (headingTC < 0)
//  {
//    headingTC = 360 + headingTC;
//  }
  
  Serial.println("Acceleration: ");
  Serial.print("X: "); Serial.print(xAccel); Serial.print("  ");
  Serial.print("Y: "); Serial.print(yAccel); Serial.print("  ");
  Serial.print("Z: "); Serial.print(zAccel); Serial.print("  ");Serial.println("m/s^2 ");
  Serial.print("X Tilt: "); Serial.println(xTilt);
  Serial.print("Y Tilt: "); Serial.println(yTilt);
  
  Serial.println("Magnetic Fields");
  Serial.print("X: "); Serial.print(mag_event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mag_event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mag_event.magnetic.z); Serial.print("  ");Serial.println("microTeslas ");
  Serial.print("Compass Heading: ");
  Serial.println(heading);
//  Serial.println("Tilt-Compensated Magnetic Fields");
//  Serial.print("X: "); Serial.print(xMagTC); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(yMagTC); Serial.println("  ");
//  Serial.print("Tilt Compensated Heading: ");
//  Serial.println(headingTC);
  
  delay(2000);
}
