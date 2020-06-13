#include "Arduino.h"
#include "TSS.h"
#include <Wire.h>

void TSS::send_cmd(char addr, byte aCMD)  //send command to sensor
{
  Wire.beginTransmission(addr);
  Wire.write(aCMD);
  Wire.endTransmission(true);
}

bool TSS::sensor_connect(char addr) { //connect to sensor
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true);
}

void TSS::read_sensor_value(char addr, unsigned long& value) { //read sensor value
  Wire.requestFrom(addr, 3);
  value = (Wire.read() << 16);
  value = value | (Wire.read() << 8);
  value = value | Wire.read();
  Wire.endTransmission(true);
}

bool TSS::init_sensor(char addr) { //initiallize sensor
  if (sensor_connect(addr)) {
    send_cmd(addr, MS5xxx_CMD_RESET);
    return true;
  } else {
    return false;
  }
}
  
void TSS::setImpactThreshold(int impactThreshold){ //setter for impactThreshold
	impactThreshold = _impactThreshold;
}

void TSS::setReleaseThreshold(int releaseThreshold){ //setter for releaseThreshold
	releaseThreshold = _releaseThreshold;
}

bool TSS::right_newval(unsigned long value_r){ //returns true or false depending on if the right sensor has a new value
	if (!right_sensor_is_reading) { //"If right sensor is NOT reading"
		right_sensor_read_start_time = micros(); //record the current system time as the time the sensor began reading
		send_cmd(0x76, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading sensor
		right_sensor_is_reading = true; //sensor reading is taking place, set the 'right_sensor_is_reading' variable to true
		return false; //returns false to 'newval' meaning the sensor doesn't have a new reading yet
  } else if (micros() - right_sensor_read_start_time > read_delay) { //if we've given the sensor enough time to update, read the values
		send_cmd(0x76, MS5xxx_CMD_ADC_READ); // read out values
		read_sensor_value(0x76, private_value_r);
		value_r = private_value_r;
		right_sensor_is_reading = false; //right sensor no longer reading, set variable as such
		return true; //returns true to newval - the sensor has had enough time and a new reading is ready!
}
}

unsigned long SmartArray::get_element(unsigned int index) {
      return arr[(index - current_index_) % array_size_];
    }
	
void SmartArray::add_element(unsigned long element) {
      current_index_++;
      current_index_ %= array_size_; //
      current_sum_ -= arr[current_index_];
      current_sum_of_differences_ -= (float(arr[(current_index_ + 1) % array_size_]) - arr[current_index_]);

      arr[current_index_] = element;

      current_sum_ += arr[current_index_];
      current_sum_of_differences_ += (float(arr[current_index_]) - arr[(current_index_ - 1) % array_size_]);
    }
	
unsigned long* SmartArray::get_array() {
      return arr;
    }
	
unsigned long SmartArray::get_average() {
      return current_sum_ / array_size_;
    }
	
unsigned long SmartArray::get_filtered_value(int filter_length) {
      unsigned long sum = 0;
      for (int i = 0; i < filter_length; i++) { //do this code 'filter length' times
        sum += get_element(-i); //sum = sum + past sensor value(i)
      }
      return sum / filter_length;
    }
	