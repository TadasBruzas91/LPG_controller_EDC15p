#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2
MCP_CAN CAN0(10);

// Setup
const uint16_t MIN_LPG_RPM = 950; // RPM
const uint16_t MAX_LPG_RPM = 3500; // RPM
const uint8_t MIN_REDUCER_TEMP = 30; // Celcius
const uint16_t MAX_LPG_EGT_TEMP = 500; // Celcius
const uint8_t LPG_INJECTION_PERCENTAGE = 10; // %
const uint8_t LPG_INJECTOR_PIN = 3;
const uint16_t LPG_INJECTOR_OPEN_TIME = 3200; // microseconds
const uint16_t LPG_INJECTOR_OPEN_TIME = 2000; // microseconds

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
uint16_t lpg_inj_duration;
bool lpg_switch = false;

// Injection //
void calculate_inj_duration(){

}

void open_injector(){

}

void close_injector(){

}

void injection(){
  if(lpg_switch && reducer_temp >= MIN_REDUCER_TEMP && rpm >= MIN_LPG_RPM && rpm <= MAX_LPG_RPM && egt_temp <= MAX_LPG_EGT_TEMP){
    iq_lpg = iq_diesel * LPG_INJECTION_PERCENTAGE / 100;
    calculate_inj_duration();
    open_injector();
  }
}
// Injection //

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
  close_injector();
  
}