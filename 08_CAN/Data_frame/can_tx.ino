#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // CS pin connected to Arduino pin 10 (change if needed)


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // Adjust bitrate and crystal frequency to match your module
                                             // Common: CAN_125KBPS, CAN_250KBPS, CAN_500KBPS
                                             // Crystal: MCP_8MHZ or MCP_16MHZ
  
  mcp2515.setNormalMode();  // Use this to actively send and receive (ACKs required on bus)
                            // Use setLoopbackMode() for testing without a real bus
  
  Serial.println("CAN Sender Ready");

  // Example: Send a standard 11-bit ID frame with 8 bytes of data every second
  canMsg.can_id  = 0x123;           // Change to your desired CAN ID
  canMsg.can_dlc = 8;               // Data length (0-8 bytes)
  canMsg.data[0] = 0x00;
  canMsg.data[1] = 0x01;
  canMsg.data[2] = 0x02;
  canMsg.data[3] = 0x03;
  canMsg.data[4] = 0x04;
  canMsg.data[5] = 0x05;
  canMsg.data[6] = 0x06;
  canMsg.data[7] = 0x07;
}

void loop() {
  
  uint8_t status = mcp2515.sendMessage(&canMsg);
  uint8_t i = 0;
  
  if (status == MCP2515::ERROR_OK) {
    Serial.println("CAN Message Sent Successfully!");
    Serial.print("ID: 0x");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] < 0x10) Serial.print("0");
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    for (i = 0; i < 8; i++){
      canMsg.data[i]++;
      if (canMsg.data[i] >= 0xF0){
        canMsg.data[i] = 0;
      }
    }
  } else {
    Serial.print("Error Sending CAN Message: ");
    Serial.println(status);
  }
  
  delay(1000);  // Send every 1 second. Adjust or remove as needed
}