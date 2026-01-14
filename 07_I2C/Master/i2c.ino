#include <Wire.h>

void setup() {
  Serial.begin(9600);

  // Start I2C as slave with address 0x40
  Wire.begin(0x40);

  // Register callbacks
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("I2C Slave Ready");
}

void loop() {
  delay(100);
}

// Called when master writes data to this slave
void receiveEvent(int numBytes) {
  Serial.print("Received: ");
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
}

// Called when master reads from this slave
void requestEvent() {
  Wire.write("OK");  // Send "OK" back
}
