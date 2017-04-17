#include <GCS_MAVLink.h>
#include <SoftwareSerial.h>

// set up the radio communication
const byte rxPin = 9;
const byte txPin = 10;
SoftwareSerial radio(rxPin, txPin);

// create placeholders for received data
mavlink_message_t receivedFCUMessage;
mavlink_status_t FCUStatus;
mavlink_message_t receivedGCSMessage;
mavlink_status_t GCSStatus;

//create placeholders for send buffers
uint8_t radioBuffer[MAVLINK_MAX_PACKET_LEN];
uint8_t usbBuffer[MAVLINK_MAX_PACKET_LEN];

// set the SYSID and COMPID
int SYSID = 20;
int COMPID = 1;

void FCUReceive() {
  while(Serial.available()) {
    uint8_t c = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &receivedFCUMessage, &FCUStatus)) {
      GCSSend();
    }
  }
}

void GCSReceive() {
    while(radio.available()) {
      uint8_t c = radio.read();
      if (mavlink_parse_char(MAVLINK_COMM_1, c, &receivedGCSMessage, &GCSStatus)) {
        FCUSend();
    }
  }
}

void FCUSend() {
  // pack the received GCS message into a buffer to send to the FCU
  int len = mavlink_msg_to_send_buffer(usbBuffer, &receivedGCSMessage);
  Serial.write(usbBuffer, len);
}

void GCSSend() {
  // pack the received FCU message into a buffer to send to the GCS
  int len = mavlink_msg_to_send_buffer(radioBuffer, &receivedFCUMessage);
  radio.write(radioBuffer, len);
}

void setup() {
  Serial.begin(115200);
  radio.begin(115200);
}

void loop() {
  FCUReceive();
  GCSReceive();
  delay(10);
}
