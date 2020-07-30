#include "TSS.h"
//#include "Wire.h"

//define two I2C buses for the 2 sets of TSS sensors
//TwoWire wire1 = TwoWire(&sercom0, 36, 37); //I2C setup for PA04 and PA05 (SDA & SCL)
//TwoWire wire2 = TwoWire(&sercom2, 38, 39); //I2C setup for PA08 and PA09
TSS* left_front;
//TSS* right_front;

void setup() {
  Serial.begin(115200);
  while (!Serial);       //Wait for Serial to begin
  Serial.println("Serial connected");
  Wire.begin();
//  wire1.begin();

//  wire1.setClock(100000);
  left_front = new TSS(&Wire, 0x76); //creates a TSS object with specified bus & address
  //  right_front = new TSS(&wire1, 0x77);
  Serial.println("Let's loop!");
}

void loop() {
  Serial.println("Looping!");
  left_front->get_new_val();
  Serial.print(left_front->get_filter_val() + " ");
  //  Serial.print(right_front->get_average());
  //  Serial.println();
  delay(100);
}
