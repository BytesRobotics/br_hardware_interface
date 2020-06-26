#include "TSS.h"
#include "Wire.h"
#define serial SerialUSB //uncomment one of these depending on if the board prints with 'SerialUSB' or 'Serial'
//#define serial Serial
TSS tube;
unsigned long start = 0;
unsigned long stoptime = 0;
void setup()
{
  serial.begin(115200);
  Wire.begin();           //begins I2C communication
  Wire.setClock(400000);  //sets I2C speed
  while (!serial);        //Wait for serial to begin

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
    serial.println(tube.filter_rightval());  //print the filtered value to remove some noise
    serial.print("  "); //spaces for the serial plotter 
    serial.print(tube.right_getaverage()); //print running average
    serial.print("  ");
    serial.print(tube.rightimpactThreshold()); //print trigger value
    serial.print("  ");
    serial.print(tube.rightreleaseThreshold()); //print trigger value
    serial.print("  ");
  }

  if (tube.left_newval())
  {
    serial.print(tube.filter_leftval());
    serial.print("  ");
    serial.print(tube.left_getaverage());
    serial.print("  ");
    serial.print(tube.leftimpactThreshold());
    serial.print("  ");
    serial.print(tube.leftreleaseThreshold());
    serial.print("  ");

  }
  return;
}

void loop()
{
  //SerialUSB.println("I'm looping!!");
  //graph_it();
  if(tube.l_impact()){
    serial.println("impact");
  }
}
