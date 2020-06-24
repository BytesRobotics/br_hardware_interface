#include "TSS.h"
#include "Wire.h"
TSS tube;
unsigned long start = 0;
unsigned long stoptime = 0;
void setup()
{
  Serial.begin(115200);
  Wire.begin();           //begins I2C communication
  Wire.setClock(400000);  //sets I2C speed
  while (!Serial);        //Wait for serial to begin

  ///initialize right and left sensors
  tube.init_sensor(0x77);
  tube.init_sensor(0x76);
  
  tube.send_cmd(0x76, MS5xxx_CMD_RESET);//reset sensors
  tube.send_cmd(0x77, MS5xxx_CMD_RESET);
  
  tube.setImpactThreshold(3000);//sets how much the pressure has to increase before an impact event is triggered
  tube.setReleaseThreshold(2500);//sets how much the pressure has to decrease before a release event is triggered

}

void graph_it() {
  if (tube.right_newval()) //if there's a new sensor value...
  {
    Serial.println(tube.filter_rightval());  //print the filtered value to remove some noise
    Serial.print("  "); //spaces for the serial plotter 
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
}

void effiecient_graph(){
  if (tube.baro_read()) //if there's a new sensor value...
  {
    Serial.println(tube.filter_rightval());  //print the filtered value to remove some noise
    Serial.print("  "); //spaces for the serial plotter
    Serial.print(tube.right_getaverage()); //print running average
    Serial.print("  ");
    Serial.print(tube.rightimpactThreshold()); //print trigger value
    Serial.print("  ");
    Serial.print(tube.rightreleaseThreshold()); //print trigger value
    Serial.print("  ");
    Serial.print(tube.filter_leftval());
    Serial.print("  ");
    Serial.print(tube.left_getaverage());
    Serial.print("  ");
    Serial.print(tube.leftimpactThreshold());
    Serial.print("  ");
    Serial.print(tube.leftreleaseThreshold());
    Serial.print("  ");

  }
}

void loop()
{
  graph_it();
}
