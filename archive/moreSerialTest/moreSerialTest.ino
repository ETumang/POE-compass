void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() == 8) {
    char rawArr[8];
    long cons[8] = {1,10,100,1000,10000,100000,1000000,10000000};
    long conv = 0;
    for (int i = 0; i < 8; i++) {
      long l = (rawArr[7-i] - '0') * cons[i];
      conv = conv + l;
    }
    Serial.println(conv * 2);
  }
}
