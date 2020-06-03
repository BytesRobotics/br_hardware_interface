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
  Wire.begin();
  Wire.setClock(400000L);
  while(!Serial);

  // Connect to sensor
  init_sensor(0x77);
  init_sensor(0x76);
}

unsigned long left_sensor_read_start_time, right_sensor_read_start_time;
bool left_sensor_is_reading=false, right_sensor_is_reading=false;
unsigned long read_delay(530);
unsigned long value_l=0, value_r=0;

#define SMART_ARRAY_SIZE 100

class SmartArray{
public:
  SmartArray(){}
  unsigned long get_element(unsigned int index){
    return arr[(index - current_index_)%array_size_];
  }
  void add_element(unsigned long element){
    current_index_++;
    current_index_ %= array_size_;
    current_sum_ -= arr[current_index_];
    current_sum_of_differences_ -= (float(arr[(current_index_+1)%array_size_]) - arr[current_index_]);
    
    arr[current_index_] = element;
    
    current_sum_ += arr[current_index_];
    current_sum_of_differences_ += (float(arr[current_index_]) - arr[(current_index_-1)%array_size_]); 
  }
  unsigned long* get_array(){
    return arr;
  }
  void print_array(){
    Serial.print("[");
    for(int i=0;i<array_size_;i++){
      Serial.print(arr[i]);
      Serial.print(", ");
    }
    Serial.println("]");
  }
  unsigned long get_average(){
    return current_sum_ / array_size_;
  }
  int get_average_slope(){
    return current_sum_of_differences_ / array_size_;
  }
private:
  unsigned long arr[SMART_ARRAY_SIZE]{0};
  const unsigned int array_size_ = SMART_ARRAY_SIZE;
  unsigned int current_index_ = 0;
  unsigned long current_sum_ = 0;
  float current_sum_of_differences_ = 0;
};

SmartArray right_history, left_history;

unsigned long last_avg_update_time= 0;
unsigned long avg_value = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if(!right_sensor_is_reading){
      right_sensor_read_start_time = micros();
      send_cmd(0x76, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);
      right_sensor_is_reading = true;
  } else if (micros()-right_sensor_read_start_time>read_delay){
      send_cmd(0x76, MS5xxx_CMD_ADC_READ); // read out values
      read_sensor_value(0x76, value_r);
      right_sensor_is_reading = false;
      right_history.add_element(value_r);
//      Serial.println(value_r);
      Serial.println(right_history.get_average());

  }

  if(!left_sensor_is_reading){
      left_sensor_read_start_time = micros();
      send_cmd(0x77, MS5xxx_CMD_ADC_CONV + MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);
      left_sensor_is_reading = true;
  } else if (micros()-left_sensor_read_start_time>read_delay){
      send_cmd(0x77, MS5xxx_CMD_ADC_READ); // read out values
      read_sensor_value(0x77, value_l);
      left_sensor_is_reading = false;
//      Serial.println(value_l);
  }

  // every two seconds update reference average
  if(micros() - last_avg_update_time > 200000){
    avg_value = right_history.get_average();
    last_avg_update_time = micros();
  }

//  Serial.println(float(right_history.get_average()) - avg_value);
  
  if(float(right_history.get_average()) - avg_value > 4000){
//    Serial.println("Collision");
  } else if (float(right_history.get_average()) - avg_value < -4000) {
//    Serial.println("Obstacle removed");
  }

//  delay(100);
  
}
