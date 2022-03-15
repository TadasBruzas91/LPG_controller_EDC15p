#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2
MCP_CAN CAN0(10);

// CANBUS read variables
long unsigned int can_id;
unsigned char len = 0;
unsigned char can_message[8];

void read_canbus(){
  if(!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&can_id, &len, can_message);
  }
}

void setup() {
  Serial.begin(250000);

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

    CAN0.setMode(MCP_NORMAL);
    pinMode(CAN0_INT, INPUT);
}

void loop() {
  
}