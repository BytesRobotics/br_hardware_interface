//Motor driver test for Byte 01
//Written for 2x Cytron MD20A brushed motor drivers
int dirpin1 = 5; //pin for direction of motor 1
int speedpin1 = 10; //pin for speed of motor 2
int dirpin2 = 0; //pin for direction of motor 2
int speed2 = 0; //pin for speed of motor 2

int sp1 = 0; //values to store speed & direction
int sp2 = 0;
bool dr1 = 0;
int dr2 = 0;
void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(9600);
  pinMode(dirpin1, OUTPUT);
  pinMode(speedpin1, OUTPUT);
  pinMode(dirpin2, OUTPUT);
  pinMode(speed2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  dr1 = (!dr1); //switch direction value
  digitalWrite(dirpin1, dr1); //set direction
  SerialUSB.print("Direction: ");
  SerialUSB.println(dr1);
  delay(50);
  
  for (int i = 0; i < 255; i++) { //ramp up motor
    analogWrite(speedpin1, i);
    delay(50);
    SerialUSB.println(i);
  }

  for (int i = 255; i > 0; i--) { //ramp down motor
    analogWrite(speedpin1, i);
    delay(50);
    SerialUSB.println(i);
  }
}
