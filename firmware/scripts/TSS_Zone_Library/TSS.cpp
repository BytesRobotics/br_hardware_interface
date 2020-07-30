#include "TSS.h"

TSS::TSS(TwoWire* wire, char addr)
{
  _wire = wire;
  Serial.println("1.0");
  _addr = addr;
  Serial.println("1.1");
  init_sensor();
}

bool TSS::is_impacted()
{
  get_new_val();
  if ((get_filter_val() > get_impact_threshold()) && ((micros() - _last_debounce_time) > _debounce_delay))
  {
    _last_debounce_time = micros(); //store system time for next time
    return true;
  }
  return false;
}

bool TSS::is_released()
{
  get_new_val();
  if ((get_filter_val() < get_release_threshold()) && ((micros() - _last_debounce_time) > _debounce_delay))
  {
    _last_debounce_time = micros(); //store system time for next time
    return true;
  }
  return false;
}

void TSS::send_cmd(byte aCMD)  //send command to sensor
{
  _wire->beginTransmission(_addr);
  _wire->write(aCMD);
  _wire->endTransmission(true);
}

bool TSS::sensor_connect() { //connect to sensor
  _wire->beginTransmission(_addr);
  Serial.println("1.2");
  return _wire->endTransmission(true);
}

void TSS::read_sensor_value(unsigned long& value) { //read sensor value
  _wire->requestFrom(_addr, 3);
  value = (_wire->read() << 16);
  value = (value | (_wire->read() << 8));
  value = (value | _wire->read());
  _wire->endTransmission(true);
}

bool TSS::init_sensor() {
  if (sensor_connect()) {
    Serial.println("1.4");
    send_cmd(MS5xxx_CMD_RESET);
    _sensor_read_start_time = micros(); //record the current system time as the time the sensors began reading
    send_cmd(MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading
    return true;
  } else {
    return false;
  }
}

void TSS::set_impact_threshold(int x) { //setter for impactThreshold
  _impact_threshold = x;
}

void TSS::set_release_threshold(int x) { //setter for releaseThreshold
  _release_threshold = x;
}

bool TSS::get_new_val() { //returns true or false depending on if the right sensor has a new value
  if (!_reading) { //"If right sensor is NOT reading"
    _sensor_read_start_time = micros(); //record the current system time as the time the sensor began reading
    send_cmd(MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading sensor
    _reading = true; //sensor reading is taking place, set the 'right_sensor_is_reading' variable to true
  } else if (micros() - _sensor_read_start_time > _read_delay) { //if we've given the sensor enough time to update, read the values
    unsigned long value;
    send_cmd(MS5xxx_CMD_ADC_READ); //sends command to read sensor
    read_sensor_value(value); //reads baro value and stores in in 'value'
    _reading = false; //sensor no longer reading, set variable as such

    if (value != 0) { //"If the sensor reading is NOT zero"
      _array.add_element(value); //add it the running average array
      return true; //returns true to newval - the sensor has had enough time and a new reading is ready!
    }
  }
  return false; //returns false to 'newval' meaning the sensor doesn't have a new reading yet
}

unsigned long TSS::get_filter_val() {
  return _array.get_filtered_value(2); //return an average of the last n reading, 2 is recommended
}

unsigned long TSS::get_average() {
  return _array.get_average();
}

unsigned long TSS::get_impact_threshold() {
  return (_impact_threshold + _array.get_average());
}

unsigned long TSS::get_release_threshold() {
  return (_array.get_average() - _release_threshold);
}
