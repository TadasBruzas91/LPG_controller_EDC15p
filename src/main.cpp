#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2
MCP_CAN CAN0(10);

// Setup
const uint16_t min_lpg_rpm = 950;
const uint16_t max_lpg_rpm = 3500;
const uint8_t min_reducer_temp = 30;
const uint16_t max_lpg_egt_temp = 500;
const uint8_t lpg_injection_percentage = 10;

// CANBUS read variables
long unsigned int can_id;
unsigned char len = 0;
unsigned char can_message[8];

// Engine variables 
uint16_t rpm, iq_diesel, iq_lpg, egt_temp;
uint8_t tps;

// LPG variables
uint8_t reducer_temp, lpg_temp;
uint16_t map_pressure, lpg_pressure;
bool lpg_switch = false;

void injection(){
  
}

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