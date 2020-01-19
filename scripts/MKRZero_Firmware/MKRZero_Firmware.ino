// Include the Adafruit GPS library with minimum version of 1.3.0
#include <Adafruit_GPS.h>
#include <Servo.h>

// https://tutorial.cytron.io/2015/04/05/using-mdd10a-arduino-uno/
// https://www.amazon.com/Cytron-Dual-Channel-Motor-Driver/dp/B07CW34YRB/ref=sr_1_6?keywords=Cytron+10A+5-30V+Dual+Channel+DC+Motor+Driver&qid=1574466884&sr=8-6
// https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
// https://www.avdweb.nl/arduino/samd21/virus

// Allows us to get byte array from float which is important for data transmission
typedef union
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;


// GPS config
// https://github.com/adafruit/Adafruit_GPS/blob/master/Adafruit_GPS.h
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);


// Distance sensor config
// The ultrasonic sensors require a 10 micro-second pulse to trigger every `timeout length seconds`
#define trig_pin 7 //only one trig pin that will handle all the sensors (PWM freq will be 1/period with dutycycle equivalent to 10us) period = timeout length
// PWM pin 2 is PA10 TCC1-W0/TCC0-W2
#define dist_timeout = 24000 //~400CM/4M max distance (time in microseconds) - 24ms - 41.67Hz - NOT Used in current implementation
// Echo pins for the 5 onboard distance sensors
#define front_dist_pin 0  //CH1
#define left_dist_pin 1   //CH2 45 degree offset from front (absolute min = 15)
#define right_dist_pin 6  //CH3 45 degree offset from front
#define rear_dist_pin 8   //CH4
#define bottom_dist_pin 9 //CH5


// Definitions for the rotary encoder
// https://youtu.be/V1txmR8GXzE
#define left_wheel_encoder_a_pin A1
#define left_wheel_encoder_b_pin 10
#define right_wheel_encoder_a_pin A2
#define right_wheel_encoder_b_pin 11
#define encoder_counts_per_revolution = 374 // Not used in current implementation

// count the total number of pulses since the start of tracking
volatile long left_wheel_pulses = 0;
volatile long right_wheel_pulses = 0;


// Control config
#define left_wheel_speed_pin 4
int left_wheel_cmd = 0;
#define left_wheel_dir_pin 2

#define right_wheel_speed_pin 5
int right_wheel_cmd = 0;
#define right_wheel_dir_pin 3

int head_servo_cmd = 0;
Servo head_servo;
#define head_servo_pin 12

//For serial event
const int packetLength = 7;         //Packet structure = |PEC|left wheel MSB|left wheel LSB|right wheel MSB|right wheel LSB|head servo MSB|head servo LSB|
byte packet[packetLength] = {0, 0, 0, 0, 0, 0, 0};   // a byte array to hold incoming data (length = 6 + PEC = 7 bytes)
boolean packetComplete = false;     // whether the string is complete

//For watchdog timer
unsigned long now = 0;
unsigned long lastPacket = 0;  //Two 16 bit ints, steering then thrptle
boolean wdt_isTripped = false; //so  the timer is not tripped continuously

//For sending packet
unsigned long lastSend = 0;
unsigned long sendPeriod = 20000; //in microsoconds (1/Hz*1000000)
const int outGoingPacketLength = 36; 
//Outgoing packet structure: |ch1 LSB|ch1 MSB|ch2 LSB|ch2 MSB|ch3 LSB|ch3 MSB|ch4 LSB|ch4 MSB|ch5 LSB|ch5 MSB|  
// |encoder_left LSB|encoder_left|encoder_left|encoder_left MSB|  
// |encoder_right LSB|encoder_right|encoder_right|encoder_right MSB|  
// |lat LSB|lat|lat|lat MSB| 
// |lon LSB|lon|lon|lon MSB| 
// |speed LSB|speed MSB|angle LSB|angle MSB|altitude LSB|altitude MSB|
// |fix|fix quality|num satelites|
// |PEC|

// For reading the controller
volatile unsigned long ch_1_rising, ch1_duty_cycle;
void ch1_rising_interrupt();
void ch1_falling_interrupt();

volatile unsigned long ch_2_rising, ch2_duty_cycle;
void ch2_rising_interrupt();
void ch2_falling_interrupt();

volatile unsigned long ch_3_rising, ch3_duty_cycle;
void ch3_rising_interrupt();
void ch3_falling_interrupt();

volatile unsigned long ch_4_rising, ch4_duty_cycle;
void ch4_rising_interrupt();
void ch4_falling_interrupt();

volatile unsigned long ch_5_rising, ch5_duty_cycle;
void ch5_rising_interrupt();
void ch5_falling_interrupt();

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
  
  // Configure left and right wheel PWM, dir, and encoder
  pinMode(right_wheel_encoder_a_pin, INPUT_PULLUP);
  pinMode(right_wheel_encoder_b_pin, INPUT_PULLUP);
  pinMode(left_wheel_encoder_a_pin, INPUT_PULLUP);
  pinMode(left_wheel_encoder_b_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(right_wheel_encoder_a_pin), right_wheel_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(left_wheel_encoder_a_pin), left_wheel_encoder_interrupt, RISING);

  pinMode(left_wheel_speed_pin, OUTPUT);
  pinMode(left_wheel_dir_pin, OUTPUT);
  pinMode(right_wheel_speed_pin, OUTPUT);
  pinMode(right_wheel_dir_pin, OUTPUT);

  head_servo.attach(head_servo_pin);
  head_servo.writeMicroseconds(1500); //Set servo to zero position

  //define pin functions for the distance sensors
  pinMode(trig_pin, OUTPUT);
  pinMode(front_dist_pin, INPUT);
  pinMode(left_dist_pin, INPUT);
  pinMode(right_dist_pin, INPUT);
  pinMode(rear_dist_pin, INPUT);
  pinMode(bottom_dist_pin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(front_dist_pin), ch1_rising_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(left_dist_pin), ch2_rising_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(right_dist_pin), ch3_rising_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rear_dist_pin), ch4_rising_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(bottom_dist_pin), ch5_rising_interrupt, RISING);  

  //Start distance sensing
  config_pwm(); //Start PWM pulses on pin 7 to control trigger pins on ultrasonic sensors

  //Other pin setup
  pinMode(LED_BUILTIN, OUTPUT);

  //Start GPS unit
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand("$PMTK251,19200*22"); //Set the GPS baud rate to 19200
  GPSSerial.flush();
  GPSSerial.end();
  GPS.begin(19200);
  
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // Request updates on antenna status
  // GPS.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  while (!Serial); //Wait to connect to computer

  lastPacket = millis(); //start watchdog timer for the first packet

}

void loop() {
 // Serial.println("CH1: " + String(ch1_duty_cycle) + " CH2: " + String(ch2_duty_cycle) + " CH3: " + String(ch3_duty_cycle) + " CH4: " + String(ch4_duty_cycle) + " CH5: " + String(ch5_duty_cycle));

  now = millis();//get current time to  ensure connection to main contorller

  if (wdt_isTripped || now - lastPacket > 500) { //If the contorller hasn't recived a new packet in half a second (short circuit limits calcs)

    left_wheel_cmd = 0;
    right_wheel_cmd = 0;

    if (wdt_isTripped == false) {
      Serial.println("Drive controller watchdog tripped!");
    }
    wdt_isTripped = true;
  }

  // When a new packet arrives indicated by a newline '\n' char:
  if (packetComplete) {      //If packet is valid
    //Packet structure = |PEC|left wheel MSB|left wheel LSB|right wheel MSB|right wheel LSB|head servo MSB|head servo LSB|
    left_wheel_cmd = calculateHardwareValues(twosComp((packet[5] << 8) | packet[4]));
    right_wheel_cmd = calculateHardwareValues(twosComp((packet[3] << 8) | packet[2]));
    head_servo_cmd = constrain(map(twosComp((packet[1] << 8) | packet[0]), -1000, 1000, 1000, 2000), 1000, 2000);

    //      Serial.println(packet, BIN);      //DEBUG
    //
    //      Serial.println(left_wheel_cmd, BIN); //DEBUG
    //      Serial.println(left_wheel_cmd);      //DEBUG
    //      Serial.println(right_wheel_cmd, BIN); //DEBUG
    //      Serial.println(right_wheel_cmd);      //DEBUG

    lastPacket = millis(); //Pet the watchdog timer
    wdt_isTripped = false;

    // clear the packet:
    for (int i = 0; i < packetLength; i++) {
      packet[i] = 0;
    }
    packetComplete = false;
  }

  // update wheels to new vlaues
  digitalWrite(left_wheel_dir_pin, (left_wheel_cmd > 0)); 
  analogWrite(left_wheel_speed_pin, abs(left_wheel_cmd));
  digitalWrite(right_wheel_dir_pin, (right_wheel_cmd > 0));
  analogWrite(right_wheel_speed_pin, abs(right_wheel_cmd));
  head_servo.writeMicroseconds(head_servo_cmd);
  
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
  
  //Send packet to computer
  if (micros() - lastSend > sendPeriod) {
    //Outgoing packet structure: |ch1 LSB|ch1 MSB|ch2 LSB|ch2 MSB|ch3 LSB|ch3 MSB|ch4 LSB|ch4 MSB|ch5 LSB|ch5 MSB|  
    // |encoder_left LSB|encoder_left|encoder_left|encoder_left MSB|  
    // |encoder_right LSB|encoder_right|encoder_right|encoder_right MSB|  
    // |lat LSB|lat|lat|lat MSB| 
    // |lon LSB|lon|lon|lon MSB| 
    // |speed LSB|speed MSB|angle LSB|angle MSB|altitude LSB|altitude MSB|
    // |fix|fix quality|num satelites|
    // |PEC|
    byte outGoingPacket[outGoingPacketLength];
    //Sends channel duty cycles to topside in microseconds for later calculation of distance
    outGoingPacket[0] = (byte)(ch1_duty_cycle & 0b0000000011111111);
    outGoingPacket[1] = (byte)(ch1_duty_cycle >> 8);
    outGoingPacket[2] = (byte)(ch2_duty_cycle & 0b0000000011111111);
    outGoingPacket[3] = (byte)(ch2_duty_cycle >> 8);
    outGoingPacket[4] = (byte)(ch3_duty_cycle & 0b0000000011111111);
    outGoingPacket[5] = (byte)(ch3_duty_cycle >> 8);
    outGoingPacket[6] = (byte)(ch4_duty_cycle & 0b0000000011111111);
    outGoingPacket[7] = (byte)(ch4_duty_cycle >> 8);
    outGoingPacket[8] = (byte)(ch5_duty_cycle & 0b0000000011111111);
    outGoingPacket[9] = (byte)(ch5_duty_cycle >> 8);
    outGoingPacket[10] = (byte)(left_wheel_pulses & 0b0000000011111111);
    outGoingPacket[11] = (byte)(left_wheel_pulses >> 8 & 0b0000000011111111);
    outGoingPacket[12] = (byte)(left_wheel_pulses >> 16 & 0b0000000011111111);
    outGoingPacket[13] = (byte)(left_wheel_pulses >> 24);
    outGoingPacket[14] = (byte)(right_wheel_pulses & 0b0000000011111111);
    outGoingPacket[15] = (byte)(right_wheel_pulses >> 8 & 0b0000000011111111);
    outGoingPacket[16] = (byte)(right_wheel_pulses >> 16 & 0b0000000011111111);
    outGoingPacket[17] = (byte)(right_wheel_pulses >> 24);

    FLOATUNION_t latitude;
    latitude.number = GPS.latitudeDegrees;
    FLOATUNION_t longitude;
    longitude.number = GPS.longitudeDegrees;
    
    outGoingPacket[18] = (byte)(latitude.bytes[0]);
    outGoingPacket[19] = (byte)(latitude.bytes[1]);
    outGoingPacket[20] = (byte)(latitude.bytes[2]);
    outGoingPacket[21] = (byte)(latitude.bytes[3]);
    outGoingPacket[22] = (byte)(longitude.bytes[0]);
    outGoingPacket[23] = (byte)(longitude.bytes[1]);
    outGoingPacket[24] = (byte)(longitude.bytes[2]);
    outGoingPacket[25] = (byte)(longitude.bytes[3]);

    //multiply speed by 100 to retain two decimal precision when moved to jetson
    outGoingPacket[26] = (byte)((int)(GPS.speed*100) & 0b0000000011111111);
    outGoingPacket[27] = (byte)((int)(GPS.speed*100) >> 8);
    outGoingPacket[28] = (byte)((int)(GPS.angle*100) & 0b0000000011111111);
    outGoingPacket[29] = (byte)((int)(GPS.angle*100) >> 8);
    outGoingPacket[30] = (byte)((int)(GPS.altitude*100) & 0b0000000011111111);
    outGoingPacket[31] = (byte)((int)(GPS.altitude*100) >> 8);
    
    outGoingPacket[32] = (byte)(GPS.fix);
    outGoingPacket[33] = (byte)(GPS.fixquality);
    outGoingPacket[34] = (byte)(GPS.satellites);
    
    byte PEC = outGoingPacket[0];
    for(int i=1; i<outGoingPacketLength-1;i++){PEC ^= outGoingPacket[i];}
    outGoingPacket[outGoingPacketLength-1] = PEC;

    Serial.write(outGoingPacket, outGoingPacketLength);
  }

  if (Serial.available() > 0) {
    serialEvent();
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void serialEvent() {

  byte pecVal = 0; //for Packet Error Checking

  for (int i = 0; i < packetLength; i++) {
    packet[i] = static_cast<byte>(Serial.read()); //Never ever use readBytes()
    if (i < packetLength-1) {
      pecVal = pecVal ^ packet[i]; //XOR with incomming byte
    }
    delayMicroseconds(1000); //Wait to finish adding the incomming byte to the buffer before reading it
  }

  //if packet is good based on PEC byte
  if (pecVal == packet[packetLength-1])
  {
    packetComplete = true;
    digitalWrite(LED_BUILTIN, HIGH);
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
