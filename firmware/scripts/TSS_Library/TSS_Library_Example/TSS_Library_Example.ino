#include "TSS.h"
#include "Wire.h"
#include "wiring_private.h"
TSS front;

void setup()
{
  front.wire1.begin();           //begins I2C communication
  front.wire1.setClock(100000);  //sets I2C speed
  //  pinPeripheral(36, PIO_SERCOM_ALT); //set wire1 pins in the MUX for sercom
  //  pinPeripheral(37, PIO_SERCOM_ALT);

  Serial.begin(115200);
  while (!Serial);        //Wait for Serial to begin
  Serial.println("wire setup");

  ///initialize right and left sensors
  front.init_sensor(0x77);
  front.init_sensor(0x76);
  Serial.println("Initialized");

  front.send_cmd(0x76, MS5xxx_CMD_RESET);//reset sensors
  front.send_cmd(0x77, MS5xxx_CMD_RESET);
  Serial.println("reset");

  front.setImpactThreshold(2000);//sets how much the pressure has to increase before an impact event is triggered
  front.setReleaseThreshold(1500);//sets how much the pressure has to decrease before a release event is triggered
}

void loop()
{
  front.graph_it();
  //  if (front.r_impact() || front.l_impact()) {
  //    Serial.println("Ur gay");
  //  }
}
