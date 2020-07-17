/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

// the setup function runs once when you press reset or power the board

const byte interruptPin = 24;
//const byte LEDpin = 13;
void setup() {
  SerialUSB.begin(9600);
  while (!SerialUSB); //Wait to connect to computer
 SerialUSB.println("serial is up and running");
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LEDpin, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, HIGH);
 
}

// the loop function runs over and over again forever
void loop() {
 /*digitalWrite(LEDpin, LOW);
 delay(1000);
 digitalWrite(LEDpin, HIGH);
 delay(1000);
 */
 SerialUSB.println("Main loop");
}

void blink() {
  //digitalWrite(LEDpin, LOW);   // turn the LED on (HIGH is the voltage level)
  SerialUSB.println("Interrupt!");

}
