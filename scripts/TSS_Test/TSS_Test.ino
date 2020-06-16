#include <Wire.h> //include I2C communication library

int impactThreshold = 3000; //threshold above average sensors value that must be exceeded to count as an impact
int releaseThreshold = 2500; //threshold below average sensor value that must be crossed to count as a release
unsigned long left_sensor_read_start_time, right_sensor_read_start_time;
bool left_sensor_is_reading = false, right_sensor_is_reading = false;
unsigned long read_delay(530); //delay to read sensor
unsigned long value_l = 0, value_r = 0;
unsigned long rightImpactTime;
unsigned long leftImpactTime;
long rightlastDebounceTime = 0;
long leftlastDebounceTime = 0;
long debounceDelay = 100000;
bool impact = false;

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

bool sensor_connect(char addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true);
}

void read_sensor_value(char addr, unsigned long& value) {
  Wire.requestFrom(addr, 3);
  value = (Wire.read() << 16);
  value = value | (Wire.read() << 8);
  value = value | Wire.read();
  Wire.endTransmission(true);
}

bool init_sensor(char addr) {
  if (sensor_connect(addr)) {
    send_cmd(addr, MS5xxx_CMD_RESET);
    return true;
  } else {
    return false;
  }
}

void setup() { //setup function, runs once on startup
  pinMode(LED_BUILTIN, OUTPUT); //sets MKRZERO onboard LED as output
  digitalWrite(LED_BUILTIN, 0); //initailize the onboard LED to off
  Serial.begin(115200); //begin serial communication to desktop at baud 115200
  Wire.begin(); //begin I2C communication
  Wire.setClock(400000); //sets I2C speed, faster speed is 400000L
  while (!Serial); //wait for serial before moving on

  // Connect to sensor
  init_sensor(0x77);
  init_sensor(0x76);
}

#define SMART_ARRAY_SIZE 2000

class SmartArray { //array class used to produce running average to compare current sensor reading to. At size 2,000, it is averaging over ~1 second
  public:
    SmartArray() {}
    unsigned long get_element(unsigned int index) {
      return arr[(index + current_index_)  % array_size_];
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
    //    unsigned long get_element(int index) {
    //      return arr[(index + current_index_) % array_size_];
    //    }
    unsigned long* get_array() {
      return arr;
    }
    void print_array() {
      Serial.print("[");
      for (int i = 0; i < array_size_; i++) {
        Serial.print(arr[i]);
        Serial.print(", ");
      }
      Serial.println("]");
    }
    unsigned long get_average() {
      return current_sum_ / array_size_;
    }

    unsigned long get_filtered_value(int filter_length) {
      unsigned long sum = 0;
      for (int i = 0; i < filter_length; i++) { //do this code 'filter length' times
        sum += get_element(-i); //sum = sum + past sensor value(i)
      }
      return sum / filter_length;
    }

    int get_average_slope() {
      return current_sum_of_differences_ / array_size_;
    }

  private:
    unsigned long arr[SMART_ARRAY_SIZE] {0};
    const unsigned int array_size_ = SMART_ARRAY_SIZE;
    unsigned int current_index_ = 0;
    unsigned long long current_sum_ = 0;
    float current_sum_of_differences_ = 0;
};

SmartArray right_history, left_history; //creates instance of SmartArray for the running average of the left and right sensors

unsigned long last_avg_update_time = 0;
unsigned long avg_value = 0;

void loop() {  //main loop begins here!!

  /*Flow:
    1-The right sensor is read, filtered, and average is calculated
    2-A light filter is applied to the raw baro reading to remove high frequency noise and have a more sensitive system
    3-Sensor data, running average, impact threshold, and release threshold are printed in a format for the serial plotter. This can be removed eventually to speed up flow
    4-2 If statments are evaluated: Has the reading gone above the impact threshold or below the release threshold?
    5-When impact occurs, record system time micros() is saved to rightImpactTime. This will be used to calculate where the impact occured later
    Steps 2 - 5 are repeated on the left sensor

  */

  // put your main code here, to run repeatedly:
  if (!right_sensor_is_reading) { //"If right sensor is NOT reading"
    right_sensor_read_start_time = micros(); //record the current system time as the time the sensor began reading
    send_cmd(0x76, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256); //send command to start reading sensor
    right_sensor_is_reading = true; //sensor reading is taking place, set the 'right_sensor_is_reading' variable to true
  } else if (micros() - right_sensor_read_start_time > read_delay) { //"If the current time minus the time the sensor began reading is greater than the time it takes to read the sensor"
    send_cmd(0x76, MS5xxx_CMD_ADC_READ); // read out values
    read_sensor_value(0x76, value_r);
    right_sensor_is_reading = false; //right sensor no longer reading, set variable as such

    if (value_r != 0) { //"If the sensor reading is NOT zero"
      right_history.add_element(value_r); //add it the running average array
            Serial.print(right_history.get_filtered_value(2));
    }

        Serial.print(" ");
        Serial.print(right_history.get_average()); //print running average of last 2000 sensor readings
        Serial.print(" ");
        Serial.print(right_history.get_average() + impactThreshold); //prints impact threshold relative to average
        Serial.print(" ");
        Serial.print(right_history.get_average() - releaseThreshold); //prints release threshold relative to running average

    if (right_history.get_filtered_value(2) >= (right_history.get_average() + impactThreshold)) { //"if the sensor reading is above the impact threshold"
      if ((micros() - rightlastDebounceTime) > debounceDelay) {
        rightImpactTime = micros(); //store the current system time, used to determine impact location later
        rightlastDebounceTime = micros();
      }
      //digitalWrite(LED_BUILTIN, 1); //turn on the onboard LED for fun
      //Serial.println("Pressed!"); //must be disabled for serial plotter
      //Serial.println(value_r - right_history.get_average()); //must be disabled for serial plotter
    }

    if (right_history.get_filtered_value(2) + releaseThreshold < (right_history.get_average())) { //"if the sensor reading is below the release threshold"
      //digitalWrite(LED_BUILTIN, 0); //turn off the onboard LED for fun
      //Serial.println("Released!"); //must be disabled for serial plotter
      //Serial.println(right_history.get_average() - value_r); //must be disabled for serial plotter
    }
  }

  //steps repeat here for other sensor
  if (!left_sensor_is_reading) {
    left_sensor_read_start_time = micros();
    send_cmd(0x77, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);
    left_sensor_is_reading = true;
  } else if (micros() - left_sensor_read_start_time > read_delay) {
    send_cmd(0x77, MS5xxx_CMD_ADC_READ); // read out values
    read_sensor_value(0x77, value_l);
    left_sensor_is_reading = false;
    //Serial.println(value_l);

    if (value_l != 0) { //"If the sensor reading is NOT zero"
      left_history.add_element(value_l); //add it the running average array
            Serial.print(" ");
            Serial.print(left_history.get_filtered_value(2));
    }

        Serial.print(" ");
        Serial.print(left_history.get_average()); //print running average of last 2000 sensor readings
        Serial.print(" ");
        Serial.print(left_history.get_average() + impactThreshold); //prints impact threshold relative to average
        Serial.print(" ");
        Serial.println(left_history.get_average() - releaseThreshold); //prints release threshold relative to running average

    if (left_history.get_filtered_value(2) > (left_history.get_average() + impactThreshold)) { //"if the sensor reading is above the impact threshold"
      if ((micros() - leftlastDebounceTime) > debounceDelay) { //impact debounce
        leftImpactTime = micros(); //store the current system time, used to determine impact location later
        leftlastDebounceTime = micros();
      }
      //Serial.println(leftImpactTime);
      digitalWrite(LED_BUILTIN, 1); //turn on the onboard LED for fun
      //Serial.println((float(rightImpactTime) - leftImpactTime) / 10000 * 343);
      //Serial.println("Pressed!"); //must be disabled for serial plotter
      //Serial.println(value_r - right_history.get_average()); //must be disabled for serial plotter
    }

    if (left_history.get_filtered_value(2) + releaseThreshold < (left_history.get_average())) { //"if the sensor reading is below the release threshold"
      digitalWrite(LED_BUILTIN, 0); //turn off the onboard LED for fun
      //Serial.println("Released!"); //must be disabled for serial plotter
      //Serial.println(right_history.get_average() - value_r); //must be disabled for serial plotter
    }
  }
  delay(1);
}
