#include "TSS.h"
#include "Wire.h"

TSS tube;

void setup()
{
  Serial.begin(115200);
  Wire.begin();           //begins I2C communication
  Wire.setClock(400000);  //sets I2C speed
  while (!Serial);        //Wait for serial to begin

  ///initialize right and left sensors
  tube.init_sensor(0x77);
  tube.init_sensor(0x76);

}

void loop()
{
  if(tube.right_newval())
  {
    Serial.println(tube.filter_rightval());
    
  }

}
