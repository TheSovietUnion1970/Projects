void setup() {
  Serial.begin(115200);
  Serial.println("UART Ready (Waiting for data...)");
}

void loop() {
  // Wait for incoming data
  if (Serial.available() > 0) {
    char c = Serial.read();       // Read a byte
    Serial.write(c);              // Echo the same byte back
  }
}
