#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // CS pin 10 (common for Arduino Uno; change if using different pin/board)

void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // Common settings: 500 kbps with 8 MHz crystal
                                              // Adjust if your module has a 16 MHz crystal: MCP_16MHZ
                                              // Common speeds: CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, etc.
  
  mcp2515.setNormalMode();  // Use setListenOnlyMode() if you only want to monitor without ACKs
  
  Serial.println("CAN Receiver Ready");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("Received CAN ID: 0x");
    Serial.print(canMsg.can_id, HEX);
    
    if (canMsg.can_id & CAN_EFF_FLAG) {
      Serial.print(" (Extended)");
    }
    if (canMsg.can_id & CAN_RTR_FLAG) {
      Serial.print(" (RTR)");
    }
    
    Serial.print(" DLC: ");
    Serial.print(canMsg.can_dlc);
    Serial.print(" Data: ");
    
    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] < 0x10) Serial.print("0");
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}