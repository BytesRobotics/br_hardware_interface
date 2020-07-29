#include "TSS.h"
#include "Wire.h"
#include "wiring_private.h"
TSS tube;

void setup()
{
  tube.wire1.begin();           //begins I2C communication
  tube.wire1.setClock(100000);  //sets I2C speed
//  pinPeripheral(36, PIO_SERCOM_ALT); //set wire1 pins in the MUX for sercom
//  pinPeripheral(37, PIO_SERCOM_ALT);

  tss_front_left = TSS(wire1, 0x76);
  Serial.begin(115200);
  while (!Serial);        //Wait for Serial to begin
  Serial.println("wire setup");

  ///initialize right and left sensors
  tube.init_sensor(0x77);
  tube.init_sensor(0x76);
  Serial.println("Initialized");

  tube.send_cmd(0x76, MS5xxx_CMD_RESET);//reset sensors
  tube.send_cmd(0x77, MS5xxx_CMD_RESET);
  Serial.println("reset");

  tube.setImpactThreshold(2000);//sets how much the pressure has to increase before an impact event is triggered
  tube.setReleaseThreshold(1500);//sets how much the pressure has to decrease before a release event is triggered
}

void graph_it() {
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

void loop()
{
  graph_it();
  //  if (tube.r_impact() || tube.l_impact()) {
  //    Serial.println("Ur gay");
  //  }
}
