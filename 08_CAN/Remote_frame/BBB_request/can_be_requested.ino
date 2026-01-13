#include <SPI.h>
#include <mcp2515.h>

#define CAN_ID 0x45  // Define the CAN ID this node responds to (11-bit standard ID)

struct can_frame receivedFrame;
MCP2515 mcp2515(10);  // Chip Select pin connected to digital pin 10 (adjust if needed)

u8 val = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial monitor to open

  Serial.println("CAN Remote Frame Receiver with Data Response");

  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // Set CAN speed to 500 kbps, assuming 8 MHz crystal (change to MCP_16MHZ if applicable)
  mcp2515.setNormalMode();

  Serial.println("MCP2515 Initialized");
}

void loop() {
  // Check for received messages
  if (mcp2515.readMessage(&receivedFrame) == MCP2515::ERROR_OK) {
    // Extract the pure ID (mask off any flags, assuming standard 11-bit ID)
    uint16_t msgId = receivedFrame.can_id & 0x7FF;

    // Check if it's a remote frame (RTR flag set) and matches our ID
    if ((receivedFrame.can_id & CAN_RTR_FLAG) && (msgId == CAN_ID)) {
      Serial.print("Received remote frame request with ID: 0x");
      Serial.print(msgId, HEX);
      Serial.print(", Requested DLC: ");
      Serial.println(receivedFrame.can_dlc);

      // Prepare the data frame response
      struct can_frame dataFrame;
      dataFrame.can_id = CAN_ID;  // Same ID, no RTR flag
      dataFrame.can_dlc = receivedFrame.can_dlc;  // Respond with the requested data length (0-8 bytes)

      // Fill the data buffer with example data (e.g., incremental bytes)
      // You can modify this to send actual data from sensors or variables
      for (uint8_t i = 0; i < dataFrame.can_dlc; i++) {
        dataFrame.data[i] = val++;  // Example: 0x01, 0x02, ..., up to DLC

        if (val >= 255) val = 0;
      }

      delay(1000);
      // Send the data frame
      if (mcp2515.sendMessage(&dataFrame) == MCP2515::ERROR_OK) {
        Serial.print("Sent data frame response with ID: 0x");
        Serial.print(CAN_ID, HEX);
        Serial.print(", DLC: ");
        Serial.print(dataFrame.can_dlc);
        Serial.print(", Data: ");
        for (uint8_t i = 0; i < dataFrame.can_dlc; i++) {
          Serial.print(dataFrame.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      } else {
        Serial.println("Error sending data frame");
      }
    } else {
      Serial.println("Received unrelated frame");
    }
  }

  // Optional: Add a small delay to avoid flooding the loop, adjust as needed
  delay(10);
}