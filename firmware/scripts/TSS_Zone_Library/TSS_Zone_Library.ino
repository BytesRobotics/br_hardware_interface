#include "TSS.h"
#include "Wire.h"

//define two I2C buses for the 2 sets of TSS sensors
//TwoWire wire1 = TwoWire(&sercom0, 36, 37); //I2C setup for PA04 and PA05 (SDA & SCL)
TwoWire wire2 = TwoWire(&sercom2, 38, 39); //I2C setup for PA08 and PA09

//TSS left_front(&wire1, 0x76); //create 4 TSS objects, 1 for each zone
//TSS right_front(&wire1, 0x77);
//TSS left_rear(&wire2, 0x76);
//TSS right_rear(&wire2, 0x77);

void setup() {
  Serial.begin(115200);
  while (!Serial);       //Wait for Serial to begin
  Serial.println("yo");
  TSS left_front(&wire2, 0x76); //create 4 TSS objects, 1 for each zone
  Serial.println(left_front.get_average());
  Serial.println("yo x2");

}

void loop() {
  //  Serial.println(left_front.get_average());
}
