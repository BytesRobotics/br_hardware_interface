/**
    Helpful links
    https://tutorial.cytron.io/2015/04/05/using-mdd10a-arduino-uno/
    https://www.amazon.com/Cytron-Dual-Channel-Motor-Driver/dp/B07CW34YRB/ref=sr_1_6?keywords=Cytron+10A+5-30V+Dual+Channel+DC+Motor+Driver&qid=1574466884&sr=8-6
    https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
 **/

// Pinmapping 
#define left_wheel_speed_pin  PA7
#define left_wheel_dir_pin PA5
#define left_wheel_encoder_a_pin PB11
#define left_wheel_encoder_b_pin PB1

#define right_wheel_speed_pin PA6      
#define right_wheel_dir_pin PA4         
#define right_wheel_encoder_a_pin PB10  
#define right_wheel_encoder_b_pin PB0   

// Allows us to get byte array from float which is important for data transmission
typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;


// Count the total number of pulses since the start of tracking
volatile long left_wheel_pulses = 0;
volatile long right_wheel_pulses = 0;

// Control config
HardwareTimer timer(3);
int left_wheel_cmd = 0;
int right_wheel_cmd = 0;

//For serial event
const int packetLength = 7;         //Packet structure = |PEC|left wheel MSB|left wheel LSB|right wheel MSB|right wheel LSB|head servo MSB|head servo LSB|
byte packet[packetLength] = {0, 0, 0, 0, 0, 0, 0};   // a byte array to hold incoming data (length = 6 + PEC = 7 bytes)
boolean packetComplete = false;     // whether the string is complete

//For watchdog timer
unsigned long now = 0;
unsigned long lastPacket = 0;
boolean wdt_isTripped = false; //so  the timer is not tripped continuously

//For sending packet
unsigned long lastSend = 0;
unsigned long sendPeriod = 15000; //in microsoconds (1/Hz*1000000)
const int outGoingPacketLength = 55;

// Interrupts for motor encoders
void right_wheel_encoder_interrupt() {
  if (digitalRead(right_wheel_encoder_b_pin)) {
    right_wheel_pulses++;
  } else {
    right_wheel_pulses--;
  }
}

void left_wheel_encoder_interrupt() {
  if (digitalRead(left_wheel_encoder_b_pin)) {
    left_wheel_pulses++;
  } else {
    left_wheel_pulses--;
  }
}

//Function for converting input (-1000 to 1000) to microseconds (1000 to 2000)
inline int calculateHardwareValues(int input) {
  input = constrain(input, -1000, 1000);
  return map(input, -1000, 1000, -255, 255); //0-255 speend and +/- for direction
}

// Go from 16 bit to 32 bit two's complement
inline int twosComp(int input) {
  if (input & 0b1000000000000000) {
    return (input | 0b11111111111111111000000000000000);
  } else {
    return (input);
  }
}

void setup() {

  Serial.begin(115200);

  // Configure left and right wheel PWM, dir, and encoder
  pinMode(right_wheel_encoder_b_pin, INPUT_PULLUP);
  pinMode(left_wheel_encoder_b_pin,  INPUT_PULLUP);
  attachInterrupt(left_wheel_encoder_a_pin, left_wheel_encoder_interrupt,  RISING);
  attachInterrupt(right_wheel_encoder_a_pin, right_wheel_encoder_interrupt, RISING); 

  pinMode(left_wheel_speed_pin,  PWM);
  pinMode(left_wheel_dir_pin,    OUTPUT);
  pinMode(right_wheel_speed_pin, PWM);
  pinMode(right_wheel_dir_pin,   OUTPUT);
  // PWM Freq = 72Mhz / prescaler / overflow
  timer.setPrescaleFactor(385);
  timer.setOverflow(255);

  pwmWrite(left_wheel_speed_pin, 0);
  pwmWrite(right_wheel_speed_pin, 0);

  //Other pin setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //high turns LED off

  while (!Serial.isConnected()); //Wait to connect to computer

  lastPacket = micros(); //start watchdog timer for the first packet
}

void loop() {

  now = micros(); //get current time to  ensure connection to main controller

  if (wdt_isTripped || now - lastPacket > 500000) { //If the contorller hasn't recived a new packet in half a second (short circuit limits calcs)
    left_wheel_cmd = 0;
    right_wheel_cmd = 0;
    wdt_isTripped = true;
  }

  // When a new packet arrives indicated by a newline '\n' char:
  if (packetComplete) { //If packet is valid
    //Packet structure = |PEC|left wheel MSB|left wheel LSB|right wheel MSB|right wheel LSB|head servo MSB|head servo LSB|
    left_wheel_cmd = calculateHardwareValues(twosComp((packet[5] << 8)  | packet[4]));
    right_wheel_cmd = calculateHardwareValues(twosComp((packet[3] << 8) | packet[2]));

    lastPacket = micros(); //Pet the watchdog timer
    wdt_isTripped = false;

    // clear the packet:
    for (int i = 0; i < packetLength; i++) {
      packet[i] = 0;
    }
    packetComplete = false;
  }

  //update wheels to new values
  digitalWrite(left_wheel_dir_pin, (left_wheel_cmd > 0));
  pwmWrite(left_wheel_speed_pin, abs(left_wheel_cmd));
  digitalWrite(right_wheel_dir_pin, (right_wheel_cmd > 0));
  pwmWrite(right_wheel_speed_pin, abs(right_wheel_cmd));

  //Send packet to computer
  if (micros() - lastSend > sendPeriod) {
    byte outGoingPacket[outGoingPacketLength];
//    outGoingPacket[0] = (byte)(ch0_duty_cycle & 0b0000000011111111);
//    outGoingPacket[1] = (byte)(ch0_duty_cycle >> 8);
//    outGoingPacket[0] = (byte)(ch1_duty_cycle & 0b0000000011111111);
//    outGoingPacket[1] = (byte)(ch1_duty_cycle >> 8);
//    outGoingPacket[2] = (byte)(ch2_duty_cycle & 0b0000000011111111);
//    outGoingPacket[3] = (byte)(ch2_duty_cycle >> 8);
//    outGoingPacket[4] = (byte)(ch3_duty_cycle & 0b0000000011111111);
//    outGoingPacket[5] = (byte)(ch3_duty_cycle >> 8);
//    outGoingPacket[6] = (byte)(ch4_duty_cycle & 0b0000000011111111);
//    outGoingPacket[7] = (byte)(ch4_duty_cycle >> 8);
//    outGoingPacket[8] = (byte)(ch5_duty_cycle & 0b0000000011111111);
//    outGoingPacket[9] = (byte)(ch5_duty_cycle >> 8);
//    outGoingPacket[10] = (byte)(ch6_duty_cycle & 0b0000000011111111);
//    outGoingPacket[11] = (byte)(ch6_duty_cycle >> 8);
//    outGoingPacket[12] = (byte)(ch7_duty_cycle & 0b0000000011111111);
//    outGoingPacket[13] = (byte)(ch7_duty_cycle >> 8);
//    outGoingPacket[14] = (byte)(ch8_duty_cycle & 0b0000000011111111);
//    outGoingPacket[15] = (byte)(ch8_duty_cycle >> 8);
//    outGoingPacket[16] = (byte)(ch9_duty_cycle & 0b0000000011111111);
//    outGoingPacket[17] = (byte)(ch9_duty_cycle >> 8);
//    outGoingPacket[18] = (byte)(ch10_duty_cycle & 0b0000000011111111);
//    outGoingPacket[19] = (byte)(ch10_duty_cycle >> 8);
//    outGoingPacket[20] = (byte)(ch11_duty_cycle & 0b0000000011111111);
//    outGoingPacket[21] = (byte)(ch11_duty_cycle >> 8);
    outGoingPacket[24] = (byte)(left_wheel_pulses & 0b0000000011111111);
    outGoingPacket[25] = (byte)(left_wheel_pulses >> 8 & 0b0000000011111111);
    outGoingPacket[26] = (byte)(left_wheel_pulses >> 16 & 0b0000000011111111);
    outGoingPacket[27] = (byte)(left_wheel_pulses >> 24);
    outGoingPacket[28] = (byte)(right_wheel_pulses & 0b0000000011111111);
    outGoingPacket[29] = (byte)(right_wheel_pulses >> 8 & 0b0000000011111111);
    outGoingPacket[30] = (byte)(right_wheel_pulses >> 16 & 0b0000000011111111);
    outGoingPacket[31] = (byte)(right_wheel_pulses >> 24);

//    FLOATUNION_t latitude;
//    latitude.number = 10.5;
//    FLOATUNION_t longitude;
//    longitude.number = 14.2;
//    FLOATUNION_t hdop; //horizontal dilution of precision for variance calculation and debugging
//    hdop.number = 1.7;

//    outGoingPacket[32] = (byte)(latitude.bytes[0]);
//    outGoingPacket[33] = (byte)(latitude.bytes[1]);
//    outGoingPacket[34] = (byte)(latitude.bytes[2]);
//    outGoingPacket[35] = (byte)(latitude.bytes[3]);
//
//    outGoingPacket[36] = (byte)(longitude.bytes[0]);
//    outGoingPacket[37] = (byte)(longitude.bytes[1]);
//    outGoingPacket[38] = (byte)(longitude.bytes[2]);
//    outGoingPacket[39] = (byte)(longitude.bytes[3]);
//
//    outGoingPacket[40] = (byte)(hdop.bytes[0]);
//    outGoingPacket[41] = (byte)(hdop.bytes[1]);
//    outGoingPacket[42] = (byte)(hdop.bytes[2]);
//    outGoingPacket[43] = (byte)(hdop.bytes[3]);
//
//    //multiply speed by 100 to retain two decimal precision when moved to jetson
//    outGoingPacket[44] = (byte)((int)(GPS.speed * 100) & 0b0000000011111111);
//    outGoingPacket[45] = (byte)((int)(GPS.speed * 100) >> 8);
//    outGoingPacket[46] = (byte)((int)(GPS.angle * 100) & 0b0000000011111111);
//    outGoingPacket[47] = (byte)((int)(GPS.angle * 100) >> 8);
//    outGoingPacket[48] = (byte)((int)(GPS.altitude * 100) & 0b0000000011111111);
//    outGoingPacket[49] = (byte)((int)(GPS.altitude * 100) >> 8);
//
//    outGoingPacket[50] = (byte)(GPS.fix);
//    outGoingPacket[51] = (byte)(GPS.fixquality);
//    outGoingPacket[52] = (byte)(GPS.satellites);
//    outGoingPacket[53] = (byte)(TSS_states);

    byte PEC = outGoingPacket[0];
    for (int i = 1; i < outGoingPacketLength - 1; i++) {
      PEC ^= outGoingPacket[i];
    }
    outGoingPacket[outGoingPacketLength - 1] = PEC;
    Serial.write(outGoingPacket, outGoingPacketLength);

    lastSend = micros();
  }

  if (Serial.available() > 0) {
    serialEvent();
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void serialEvent() {

  byte pecVal = 0; //for Packet Error Checking

  for (int i = 0; i < packetLength; i++) {
    packet[i] = static_cast<byte>(Serial.read()); //Never ever use readBytes()
    if (i < packetLength - 1) {
      pecVal = pecVal ^ packet[i]; //XOR with incomming byte
    }
    delayMicroseconds(1000); //Wait to finish adding the incomming byte to the buffer before reading it
  }

  //if packet is good based on PEC byte
  if (pecVal == packet[packetLength - 1])
  {
    packetComplete = true;
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    while (Serial.available()) {
      Serial.read(); //Clear the Serial input buffer
    }
    // clear the packet:
    for (int i = 0; i < packetLength; i++) {
      packet[i] = 0;
    }
  }
}
