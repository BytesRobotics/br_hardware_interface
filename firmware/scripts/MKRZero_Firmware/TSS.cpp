#include "Arduino.h"
#include "TSS.h"

void TTS_graph_it(TSS& tube) {
  if (tube.right_newval()) //if there's a new sensor value...
  {
    Serial.println(tube.filter_rightval());  //print the filtered value to remove some noise
    Serial.print("  "); //spaces for the Serial plotter
    Serial.print(tube.right_getaverage()); //print running average
    Serial.print("  ");
    Serial.print(tube.rightimpactThreshold()); //print trigger value
    Serial.print("  ");
    Serial.print(tube.rightreleaseThreshold()); //print trigger value
    Serial.print("  ");
  }

  if (tube.left_newval())
  {
    Serial.print(tube.filter_leftval());
    Serial.print("  ");
    Serial.print(tube.left_getaverage());
    Serial.print("  ");
    Serial.print(tube.leftimpactThreshold());
    Serial.print("  ");
    Serial.print(tube.leftreleaseThreshold());
    Serial.print("  ");

  }
  return;
}

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

SmartArray left, right; //two instances of the SmartArray to handle the left and right sensors

bool TSS::r_impact()
{
  right_newval();
  if ((filter_rightval() > rightimpactThreshold()) && ((micros() - rightlastDebounceTime) > debounceDelay))
  {
    rightlastDebounceTime = micros(); //store system time for next time
    //Serial.println(rightlastDebounceTime);
    return true;
  }
  return false;
}

bool TSS::l_impact()
{
  left_newval();
  if ((filter_leftval() > leftimpactThreshold()) && ((micros() - leftlastDebounceTime) > debounceDelay))
  {
    leftlastDebounceTime = micros(); //store system time for next time
    //Serial.println(leftlastDebounceTime);
    return true;
  }
  return false;
}

bool TSS::r_release()
{
  right_newval();
  if ((filter_rightval() < rightreleaseThreshold()) && ((micros() - rightlastDebounceTime) > debounceDelay))
  {
    rightlastDebounceTime = micros(); //store system time for next time
    //Serial.println(rightlastDebounceTime);
    return true;
  }
  return false;
}

bool TSS::l_release()
{
  left_newval();
  if ((filter_leftval() < leftreleaseThreshold()) && ((micros() - leftlastDebounceTime) > debounceDelay))
  {
    leftlastDebounceTime = micros(); //store system time for next time
    //Serial.println(leftlastDebounceTime);
    return true;
  }
  return false;
}

void TSS::send_cmd(char addr, byte aCMD)  //send command to sensor
{
  wire2.beginTransmission(addr);
  wire2.write(aCMD);
  wire2.endTransmission(true);
}

bool TSS::sensor_connect(char addr) { //connect to sensor
  wire2.beginTransmission(addr);
  return wire2.endTransmission(true);
}

void TSS::read_sensor_value(char addr, unsigned long& value) { //read sensor value
  wire2.requestFrom(addr, 3);
  value = (wire2.read() << 16);
  value = value | (wire2.read() << 8);
  value = value | wire2.read();
  wire2.endTransmission(true);
}

bool TSS::init_sensor(char addr) { //initialize sensor
  if (sensor_connect(addr)) {
    send_cmd(addr, MS5xxx_CMD_RESET);
    sensor_read_start_time = micros(); //record the current system time as the time the sensors began reading
    send_cmd(0x76, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading right sensor
    send_cmd(0x77, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //read left sensor
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

bool TSS::right_newval() { //returns true or false depending on if the right sensor has a new value
  if (!right_sensor_is_reading) { //"If right sensor is NOT reading"
    right_sensor_read_start_time = micros(); //record the current system time as the time the sensor began reading
    send_cmd(0x76, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading sensor
    right_sensor_is_reading = true; //sensor reading is taking place, set the 'right_sensor_is_reading' variable to true
    return false; //returns false to 'newval' meaning the sensor doesn't have a new reading yet
  } else if (micros() - right_sensor_read_start_time > read_delay) { //if we've given the sensor enough time to update, read the values
    send_cmd(0x76, MS5xxx_CMD_ADC_READ); //sends command to read sensor
    read_sensor_value(0x76, value_r); //reads baro value and stores in in value_r
    right_sensor_is_reading = false; //right sensor no longer reading, set variable as such

    if (value_r != 0) { //"If the sensor reading is NOT zero"
      right.add_element(value_r); //add it the running average array
      return true; //returns true to newval - the sensor has had enough time and a new reading is ready!
    }
  }
}

bool TSS::left_newval() {
  if (!left_sensor_is_reading) {
    left_sensor_read_start_time = micros();
    send_cmd(0x77, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);
    left_sensor_is_reading = true;
    return false;
  } else if (micros() - left_sensor_read_start_time > read_delay) {
    send_cmd(0x77, MS5xxx_CMD_ADC_READ); // read out values
    read_sensor_value(0x77, value_l);
    left_sensor_is_reading = false;

    if (value_l != 0) { //"If the sensor reading is NOT zero"
      left.add_element(value_l); //add it the running average array
      return true;
    }
  }
}

unsigned long TSS::filter_rightval() {
  return right.get_filtered_value(2);
}

unsigned long TSS::right_getaverage() {
  return right.get_average();
}

unsigned long TSS::filter_leftval() {
  return left.get_filtered_value(2);
}

unsigned long TSS::left_getaverage() {
  return left.get_average();
}

unsigned long TSS::rightimpactThreshold() {
  return (_impactThreshold + right.get_average());
}

unsigned long TSS::leftimpactThreshold() {
  return (_impactThreshold + left.get_average());
}

unsigned long TSS::rightreleaseThreshold() {
  return (right.get_average() - _releaseThreshold);
}

unsigned long TSS::leftreleaseThreshold() {
  return (left.get_average() - _releaseThreshold);
}
