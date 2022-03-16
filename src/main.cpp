#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2
MCP_CAN CAN0(10);

// Setup //
const uint16_t MIN_LPG_RPM = 950; // RPM
const uint16_t MAX_LPG_RPM = 3500; // RPM
const uint8_t MIN_REDUCER_TEMP = 30; // Celcius
const uint16_t MAX_LPG_EGT_TEMP = 500; // Celcius
const uint8_t LPG_INJECTION_PERCENTAGE = 10; // %
const uint16_t LPG_INJECTOR_OPEN_TIME = 3200; // microseconds
const uint16_t LPG_INJECTOR_CLOSE_TIME = 2000; // microseconds
const uint8_t LPG_INJECTOR_PIN = 3;
const uint8_t DIESEL_INJECTOR_INPUT_OUTPUT = 4;
const uint8_t MAP_SENSOR_PIN = 0;
const uint8_t LPG_PRESSURE_SENSOR_PIN = 0;
const uint8_t REDUCER_TEMP_SENSOR_PIN = 0;
const uint8_t LPG_TEMP_SENSOR_PIN = 0;
const uint8_t LPG_TANK_LEVEL_PIN = 0;

// CANBUS read variables //
long unsigned int can_id;
unsigned char can_msg_len = 0;
unsigned char can_message[8];

// Engine variables //
uint16_t rpm, iq_diesel, iq_lpg, egt_temp;
uint8_t tps;

// LPG variables //
uint8_t reducer_temp, lpg_temp;
uint16_t map_pressure, lpg_pressure;
uint8_t lpg_tank_level;
uint16_t lpg_inj_duration; // micro seconds
bool lpg_switch = false;

// Time variables
unsigned long current_time_millis;
unsigned long current_time_micros;

// Sensors read variables //
unsigned long sensors_read_time = 0;
uint8_t sensors_read_interval = 100; // ms

int interpolation(int x1, int x2, int x3, int y1, int y3){
  int y2 = (x2-x1)*(y3-y1)/(x3-x1)+y1;
  return y2;
}

// All controller sensors //
void read_map(){
  map_pressure;
  lpg_pressure;
}

void read_lpg_temp(){
  lpg_temp;
}

void read_reducer_temp(){
  reducer_temp;
}

void read_egt_temp(){
  egt_temp;
}

void read_lpg_tank_level(){
  lpg_tank_level;
}

void read_sensors(){
  if((current_time_millis - sensors_read_time) >= sensors_read_interval){
    read_map();
    read_lpg_temp();
    read_reducer_temp();
    read_lpg_tank_level();
    sensors_read_time = millis();
  }
}

// All controller sensors //

// Injection start //
void calculate_inj_duration(){
  lpg_inj_duration;
}

void open_injector(){
  digitalWrite(LPG_INJECTOR_PIN, HIGH);
}

void close_injector(){
  digitalWrite(LPG_INJECTOR_PIN, LOW);
}

void injection(){
  if(lpg_switch && reducer_temp >= MIN_REDUCER_TEMP && rpm >= MIN_LPG_RPM && rpm <= MAX_LPG_RPM && egt_temp <= MAX_LPG_EGT_TEMP){
    iq_lpg = iq_diesel * LPG_INJECTION_PERCENTAGE / 100;
    calculate_inj_duration();
    open_injector();
  }
}
// Injection end//

void read_canbus(){
  if(!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&can_id, &can_msg_len, can_message);
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
  current_time_millis = millis();
  current_time_micros = micros();
  close_injector();
  read_canbus();
  read_sensors();
}