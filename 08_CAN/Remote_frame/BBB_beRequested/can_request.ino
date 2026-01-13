#include <SPI.h>
#include <mcp2515.h>

#define CAN_ID 0x45

struct can_frame canRemoteFrame;
struct can_frame canReceivedFrame;
MCP2515 mcp2515(10); // Chip Select pin connected to digital pin 10 (adjust if needed)

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor to open

  Serial.println("CAN Remote Frame Sender with Receiver");

  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Set CAN speed to 500 kbps, assuming 8 MHz crystal (change to MCP_16MHZ if applicable)
  mcp2515.setNormalMode();

  Serial.println("MCP2515 Initialized");
}

void loop() {
  // Prepare and send the remote frame
  canRemoteFrame.can_id = CAN_ID | CAN_RTR_FLAG; // Standard 11-bit ID (0x123), set RTR flag for remote request
  canRemoteFrame.can_dlc = 4; // Request 4 bytes of data (DLC can be 0-8)

  if (mcp2515.sendMessage(&canRemoteFrame) == MCP2515::ERROR_OK) {
    Serial.println("Remote frame sent successfully");
  } else {
    Serial.println("Error sending remote frame");
  }

  // Check for received messages (poll for response)
  if (mcp2515.readMessage(&canReceivedFrame) == MCP2515::ERROR_OK) {
    // Verify it's a data frame response (RTR=0, same ID)
    if (canReceivedFrame.can_id == CAN_ID && !(canReceivedFrame.can_id & CAN_RTR_FLAG)) {
      Serial.print("Received data frame with ID: 0x");
      Serial.print(canReceivedFrame.can_id, HEX);
      Serial.print(", DLC: ");
      Serial.print(canReceivedFrame.can_dlc);
      Serial.print(", Data: ");

      for (int i = 0; i < canReceivedFrame.can_dlc; i++) {
        Serial.print(canReceivedFrame.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Received unrelated frame");
    }
  }

  delay(2000); // Send remote frame every 1 second (adjust as needed)
}