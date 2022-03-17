#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// CAN BUS module variables
#define CAN0_INT 2
MCP_CAN CAN0(10);
unsigned long counter = 0;

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
const uint8_t REDUCER_TEMP_SENSOR_PIN = A0;
const uint8_t LPG_TEMP_SENSOR_PIN = A1;
const uint8_t LPG_TANK_LEVEL_PIN = 0;

// Serial out variables //
bool serial_out_on = true;
uint8_t serial_out_mode = 2;
unsigned long serial_out_send = 0;
const long serial_send_interval = 250;

// CANBUS read variables //
long unsigned int can_id;
unsigned char can_msg_len = 0;
unsigned char can_message[8];

// Engine variables //
uint16_t rpm, iq_diesel, iq_lpg, egt_temp;
uint8_t tps, speed, diesel_level;
uint8_t coolant_temp, oil_temp, outside_temp;
bool low_diesel_level_warning;

// LPG variables //
uint8_t reducer_temp, lpg_temp;
uint16_t map_pressure, lpg_pressure;
uint8_t lpg_tank_level;
uint16_t lpg_inj_duration; // micro seconds
bool lpg_switch = false;

// Time variables //
unsigned long current_time_millis;
unsigned long current_time_micros;

// Sensors read variables //
unsigned long sensors_read_time = 0;
uint8_t sensors_read_interval = 250; // ms

// Temperature sensors variables
float Ro = 2.2; // Nominal resistance
uint16_t B =  3500; // Beta constant
float Rseries = 2.2;// Series resistor 10K
float To = 298.15; // Nominal Temperature

int interpolation(int x1, int x2, int x3, int y1, int y3){
  int y2 = (x2-x1)*(y3-y1)/(x3-x1)+y1;
  return y2;
}

uint8_t ntc_thermistor(uint16_t analog_input){
  float Vi = analog_input * (5.0 / 1023.0);
  float R = (Vi * Rseries) / (5 - Vi);
  float T =  1 / ((1 / To) + ((log(R / Ro)) / B));
  return T - 273.15; // Converting kelvin to celsius
}

// All controller sensors //
void read_map(){
  map_pressure;
  lpg_pressure;
}

void read_lpg_temp(){
  lpg_temp = ntc_thermistor(analogRead(LPG_TEMP_SENSOR_PIN));
}

void read_reducer_temp(){
  reducer_temp = ntc_thermistor(analogRead(REDUCER_TEMP_SENSOR_PIN));
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

// CAN BUS //

void getRpm(){
  rpm = (can_message[2]+256*can_message[3])/4;
}

void getTps(){
  tps = can_message[5] * 4 / 10;
}

void getIq(){
  iq_diesel = can_message[1] * 25 / 10;
}

void getSpeed(){
  speed = can_message[3] * 129 / 100;
}

void get_coolant_temp(){
  coolant_temp = can_message[4] * 75 / 100 - 48;
}

void get_oil_temp(){
  oil_temp = can_message[3] * 75 / 100 - 48;
}

void get_outside_temp(){
  outside_temp = (can_message[1] - 100) / 2 ;
}

void get_fuel_level(){
  diesel_level = can_message[2] & 0x7F;
  low_diesel_level_warning = can_message[2] & 0x80 == 0x80;
}

void read_canbus(){
  if(!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&can_id, &can_msg_len, can_message);
    if(can_id == 0x280){
      getRpm();
      getTps();
      getIq();
    }

    if(can_id == 0x288){
      getSpeed();
    }

    if(can_id == 0x420){
      get_coolant_temp();
      get_oil_temp();
      get_outside_temp();
    }

    if(can_id == 0x320){
      get_fuel_level();
    }
  }
}
// CAN BUS end//

// Serial output //
void serial_output(){
  if(serial_out_on && (current_time_millis - serial_out_send) >= serial_send_interval){
    if(serial_out_mode == 1){
      Serial.print(rpm);
      Serial.print(F("\t"));
      Serial.print(iq_diesel);
      Serial.print(F("\t"));
      Serial.print(iq_lpg);
      Serial.print(F("\t"));
      Serial.print(tps);
      Serial.print(F("\t"));
      Serial.print(speed);
      Serial.print(F("\t"));
      Serial.print(coolant_temp);
      Serial.print(F("\t"));
      Serial.print(oil_temp);
      Serial.print(F("\t"));
      Serial.print(diesel_level);
      Serial.print(F("\t"));
      Serial.println(outside_temp);
    }else if(serial_out_mode == 2){
      Serial.print(reducer_temp);
      Serial.print(F("\t"));
      Serial.println(lpg_temp);
    }
    serial_out_send = current_time_millis;
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
  serial_output();
}