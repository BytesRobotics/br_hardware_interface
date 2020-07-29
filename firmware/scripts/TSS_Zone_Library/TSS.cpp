#include "Arduino.h"
#include "TSS.h"

//class SmartArray is put int the .cpp because it only needs to be accessed internally by the library
//and not by the end user
class SmartArray { //array class used to produce running average to compare current sensor reading to. At size 2,000, it is averaging over ~1 second
  public:
    SmartArray() { }
    unsigned long get_element(unsigned int index) {
      return arr[(index + current_index_) % array_size_];
    }
    void add_element(unsigned long element) {
      current_index_++;
      current_index_ %= array_size_; //
      current_sum_ -= arr[current_index_];
      current_sum_of_differences_ -= (float(arr[(current_index_ + 1) % array_size_]) - arr[current_index_]);

      arr[current_index_] = element;

      current_sum_ += arr[current_index_];
      current_sum_of_differences_ += (float(arr[current_index_]) - arr[(current_index_ - 1) % array_size_]);
    }

    unsigned long* get_array() {
      return arr;
    }

    unsigned long get_average() {
      return current_sum_ / array_size_;
    }

    unsigned long get_filtered_value(int filter_length) {
      //      unsigned long sum = 0;
      //      for (int i = 0; i < 2; i++) { //do this code 'filter length' times
      //        sum += get_element(-1); //sum = sum + past sensor value(i)
      //      }
      //      return sum / 2;
      return get_element(0);
    }

  private:
    unsigned long arr[SMART_ARRAY_SIZE] {0};
    const unsigned int array_size_ = SMART_ARRAY_SIZE;
    unsigned int current_index_ = 0;
    unsigned long long current_sum_ = 0;
    float current_sum_of_differences_ = 0;
};

SmartArray running_average_array; //instance of the SmartArray to handle the sensor values

TSS::TSS(TwoWire* wire, uint8_t addr) {
  _wire = wire;
  _addr = addr;
  send_cmd(MS5xxx_CMD_RESET); //resets sensor upon object creation
}

bool TSS::_impact()
{
  newval();
  if ((filter_val() > impact_Threshold()) && ((micros() - lastDebounceTime) > debounceDelay))
  {
    lastDebounceTime = micros(); //store system time for next time
    return true;
  }
  return false;
}

bool TSS::_release()
{
  newval();
  if ((filter_val() < release_Threshold()) && ((micros() - lastDebounceTime) > debounceDelay))
  {
    lastDebounceTime = micros(); //store system time for next time
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
  return _wire->endTransmission(true);
}

void TSS::read_sensor_value(unsigned long& value) { //read sensor value
  _wire->requestFrom(_addr, 3);
  value = (_wire->read() << 16);
  value = (value | (_wire->read() << 8));
  value = (value | _wire->read());
  _wire->endTransmission(true);
}

bool TSS::init_sensor() { //initialize sensor
  if (sensor_connect()) {
    send_cmd(MS5xxx_CMD_RESET);
    sensor_read_start_time = micros(); //record the current system time as the time the sensors began reading
    send_cmd(MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading right sensor
    return true;
  } else {
    return false;
  }
}

void TSS::setImpactThreshold(int x) { //setter for impactThreshold
  _impactThreshold = x;
}

void TSS::setReleaseThreshold(int x) { //setter for releaseThreshold
  _releaseThreshold = x;
}

bool TSS::newval() { //returns true or false depending on if the right sensor has a new value
  if (!reading) { //"If right sensor is NOT reading"
    sensor_read_start_time = micros(); //record the current system time as the time the sensor began reading
    send_cmd(MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading sensor
    reading = true; //sensor reading is taking place, set the 'right_sensor_is_reading' variable to true
    return false; //returns false to 'newval' meaning the sensor doesn't have a new reading yet
  } else if (micros() - sensor_read_start_time > read_delay) { //if we've given the sensor enough time to update, read the values
    send_cmd(MS5xxx_CMD_ADC_READ); //sends command to read sensor
    read_sensor_value(value); //reads baro value and stores in in 'value'
    reading = false; //sensor no longer reading, set variable as such

    if (value != 0) { //"If the sensor reading is NOT zero"
      running_average_array.add_element(value); //add it the running average array
      return true; //returns true to newval - the sensor has had enough time and a new reading is ready!
    }
  }
}

unsigned long TSS::filter_val() {
  return running_average_array.get_filtered_value(2); //return an average of the last n reading, 2 is recommended
}

unsigned long TSS::get_average() {
  return running_average_array.get_average();
}

unsigned long TSS::impact_Threshold() {
  return (_impactThreshold + running_average_array.get_average());
}

unsigned long TSS::release_Threshold() {
  return (running_average_array.get_average() - _releaseThreshold);
}
