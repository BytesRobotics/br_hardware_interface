#include <Wire.h>

#define MS5xxx_CMD_RESET    0x1E    // perform reset
#define MS5xxx_CMD_ADC_READ 0x00    // initiate read sequence
#define MS5xxx_CMD_ADC_CONV 0x40    // start conversion
#define MS5xxx_CMD_ADC_D1   0x00    // read ADC 1
#define MS5xxx_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256

void send_cmd(char addr, byte aCMD)
{
  Wire.beginTransmission(addr);
  Wire.write(aCMD);
  Wire.endTransmission(true);
}

bool sensor_connect(char addr){
  Wire.begin();
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true);
}

void read_sensor_value(char addr, unsigned long& value){
  Wire.requestFrom(addr, 3);
  value = (Wire.read()<<16);
  value = value | (Wire.read()<<8);
  value = value | Wire.read();
  Wire.endTransmission(true);
}

bool init_sensor(char addr){
  if(sensor_connect(addr)){
    send_cmd(addr, MS5xxx_CMD_RESET);
    return true;
  } else {
    return false;
  }
}


void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Connect to sensor
  init_sensor(0x77);
  init_sensor(0x76);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long value_l=0, value_r=0;

  send_cmd(0x76, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);
  send_cmd(0x77, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);
  delayMicroseconds(300);
  send_cmd(0x76, MS5xxx_CMD_ADC_READ); // read out values
  send_cmd(0x77, MS5xxx_CMD_ADC_READ); // read out values

  read_sensor_value(0x77, value_l);
  read_sensor_value(0x76, value_r);

  
  Serial.print(value_l);
  Serial.print(" ");
  Serial.println(value_r);
}
