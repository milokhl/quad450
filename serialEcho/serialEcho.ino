#include <SoftwareSerial.h>

// set up the radio communication
const byte rxPin = 9;
const byte txPin = 10;
SoftwareSerial radio(rxPin, txPin);

int incomingByte = 0;

void setup() {
  Serial.begin(115200);
  radio.begin(115200);
}

void loop() {
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    radio.write(incomingByte);
  }
  
}
