#include <MS5xxx.h>
#include <Wire.h>

#define MS5xxx_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256

class MS560702: public MS5xxx{
public:
    MS560702(char aAddr) : MS5xxx(&Wire){
      setI2Caddr(aAddr);
    }
    
    void LightRead() {
      unsigned long D1=0;
      D1=read_adc(MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_256);
      P=D1;
    }

};


MS560702 sensor_l(0x77);
MS560702 sensor_r(0x76);


void setup() {
  Serial.begin(115200);

  while(!Serial);
  
  if(sensor_l.connect()>0) {
    Serial.println("Error connecting...");
    delay(500);
  } else {
    sensor_l.ReadProm();
  }

  if(sensor_r.connect()>0) {
    Serial.println("Error connecting...");
    delay(500);
  } else {
    sensor_r.ReadProm();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor_l.LightRead();
  sensor_r.LightRead();
  Serial.print(sensor_l.GetPres());
  Serial.print(" ");
  Serial.println(sensor_r.GetPres());

}
