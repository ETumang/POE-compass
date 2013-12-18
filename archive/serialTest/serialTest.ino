void setup()
{
  Serial.begin(9600);
}
long d;
void loop()
{
  if(Serial.available() >= 4){
    char c = Serial.read();
    if(c) {
      d = c;
      Serial.println(d);
    }
  }
}
