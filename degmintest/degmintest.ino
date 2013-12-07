void setup(){
  Serial.begin(115200);
  Serial.println("Hi");
}
void loop(){
  long raw = 4579.7543 * 10000;
  float degrees = raw / 1000000;
  float minutes = (raw % 1000000);
  minutes = minutes / 600000;
  degrees = (degrees) + minutes;
  Serial.println(raw);
  Serial.println(degrees);
  delay(5000);
}
