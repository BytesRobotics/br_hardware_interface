// Include the Adafruit GPS library with minimum version of 1.3.0
#include "TSS.h"
#include "Wire.h"
#include <Adafruit_GPS.h>
#include <Servo.h>

/**
    Helpful links
    https://tutorial.cytron.io/2015/04/05/using-mdd10a-arduino-uno/
    https://www.amazon.com/Cytron-Dual-Channel-Motor-Driver/dp/B07CW34YRB/ref=sr_1_6?keywords=Cytron+10A+5-30V+Dual+Channel+DC+Motor+Driver&qid=1574466884&sr=8-6
    https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
 **/

// Allows us to get byte array from float which is important for data transmission
typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// Modified attachInterrupt() function that allows us to specify port and pad number, defined in CustomInterrupts.ino
// Port, pin, interrupt number (ex: PA17 is 0, 17, 1)
void attachInterrupt(uint32_t port, uint32_t pin, uint32_t extint, voidFuncPtr callback, uint32_t mode);

// TSS Config
TSS tube; //create an instance of the TSS library for each section of our tube
unsigned long lastloop = 0;
int TSS_states = 0;

// GPS config
// https://github.com/adafruit/Adafruit_GPS/blob/master/Adafruit_GPS.h
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

// Distance sensor config
// The ultrasonic sensors require a 10 micro-second pulse to trigger every `timeout length seconds`
#define trig_pin 7 //only one trig pin that will handle all the sensors (PWM freq will be 1/period with dutycycle equivalent to 10us) period = timeout length
// PWM pin 2 is PA10 TCC1-W0/TCC0-W2
#define dist_timeout = 24000 //~400CM/4M max distance (time in microseconds) - 24ms - 41.67Hz - NOT Used in current implementation

// Definitions for the rotary encoder
// https://youtu.be/V1txmR8GXzE
#define left_wheel_encoder_b_pin 20  //PA06
// a pin is on PA12
#define right_wheel_encoder_b_pin 15 //PA02
// a pin is on PB8
// count the total number of pulses since the start of tracking
volatile long left_wheel_pulses = 0;
volatile long right_wheel_pulses = 0;

// Control config
#define left_wheel_speed_pin 3 //PA11
#define left_wheel_dir_pin 24  //PA18
int left_wheel_cmd = 0;

#define right_wheel_speed_pin 2 //PA10
#define right_wheel_dir_pin 10  //PA19
int right_wheel_cmd = 0;

//int head_servo_cmd = 0;
//Servo head_servo;
//#define head_servo_pin 12 //no servo on bytes

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
unsigned long sendPeriod = 20000; //in microsoconds (1/Hz*1000000)
const int outGoingPacketLength = 55;

// Forward definitions for interrupt functions
volatile unsigned long ch_0_rising, ch0_duty_cycle;
void ch0_rising_interrupt();
void ch0_falling_interrupt();

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

volatile unsigned long ch_6_rising, ch6_duty_cycle;
void ch6_rising_interrupt();
void ch6_falling_interrupt();

volatile unsigned long ch_7_rising, ch7_duty_cycle;
void ch7_rising_interrupt();
void ch7_falling_interrupt();

void left_wheel_encoder_interrupt();
void right_wheel_encoder_interrupt();

//starts PWM for ultrasonic trig pin
void config_pwm();

//Function for converting input (-1000 to 1000) to microseconds (1000 to 2000)
inline int calculateHardwareValues(int input) {
  input = constrain(input, -1000, 1000);
  return map(input, -1000, 1000, -4095, 4095); //0-4095 speend and +/- for direction
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
  analogWriteResolution(12);

  // Initialize TSS sensors
  //  tube.wire1.begin(); //begin I2C communication for TSS
  //  tube.wire1.setClock(400000);
  //  tube.init_sensor(0x77);
  //  tube.init_sensor(0x76);
  //
  //  tube.send_cmd(0x76, MS5xxx_CMD_RESET);//reset sensors
  //  tube.send_cmd(0x77, MS5xxx_CMD_RESET);
  //
  //  tube.setImpactThreshold(3000);//sets how much the pressure has to increase before an impact event is triggered
  //  tube.setReleaseThreshold(2500);//sets how much the pressure has to decrease before a release event is triggered

  // Configure left and right wheel PWM, dir, and encoder
  pinMode(right_wheel_encoder_b_pin, INPUT_PULLUP);
  pinMode(left_wheel_encoder_b_pin,  INPUT_PULLUP);
  attachInterrupt(1, 8, 8, right_wheel_encoder_interrupt,  RISING); //Wheel encoder A on PB08
  attachInterrupt(0, 12, 12, left_wheel_encoder_interrupt, RISING); //Ender B on PA12

  pinMode(left_wheel_speed_pin,  OUTPUT);
  pinMode(left_wheel_dir_pin,    OUTPUT);
  pinMode(right_wheel_speed_pin, OUTPUT);
  pinMode(right_wheel_dir_pin,   OUTPUT);

  //head_servo.attach(head_servo_pin);
  //head_servo.writeMicroseconds(1500); //Set servo to zero position

  //define pin functions for the distance sensors
  pinMode(trig_pin, OUTPUT);
  attachInterrupt(1, 6, 6, ch0_rising_interrupt,    RISING); //PB06 EXTINT6 y
  attachInterrupt(1, 7, 7, ch1_rising_interrupt,    RISING); //PB07 EXTINT7 y
  attachInterrupt(1, 10, 10, ch2_rising_interrupt,  RISING); //PB10 EXTINT10 y
  attachInterrupt(1, 11, 11, ch3_rising_interrupt,   RISING); //PB03 EXTINT11 y
  attachInterrupt(0, 20, 4, ch4_rising_interrupt,   RISING); //PA20 EXTINT4 y
  attachInterrupt(1, 5, 5, ch5_rising_interrupt,    RISING); //PB05 EXTINT5 y
  attachInterrupt(0, 16, 0, ch6_rising_interrupt,   RISING); //PA16 EXTINT0 y
  attachInterrupt(1, 2, 2, ch7_rising_interrupt,    RISING); //PB02 EXTINT2 y
  //Start distance sensing
  config_pwm();

  //Other pin setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //high turns LED off

  //Start GPS unit
  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand("$PMTK251,19200*22"); //Set the GPS baud rate to 19200
  GPSSerial.flush();
  GPSSerial.end();
  GPS.begin(19200);

  // http://aprs.gids.nl/nmea/
  // Turn on RMC (recommended minimum) and GGA (fix data including HDOP) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate (this is only the echo rate)
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ); // Fix at 1Hz to work with SBAS and Easy Mode (see PMTK datasheet for more info)

  //  // Enable all GPS correction data types SBAS and WAAS
  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PMTK_ENABLE_WAAS);

  while (!Serial); //Wait to connect to computer

  lastPacket = micros(); //start watchdog timer for the first packet
}

void loop() {

  SerialUSB.println(ch0_duty_cycle);
  SerialUSB.println(ch1_duty_cycle);
  SerialUSB.println(ch2_duty_cycle);
  SerialUSB.println(ch3_duty_cycle);
  SerialUSB.println(ch4_duty_cycle);
  SerialUSB.println(ch5_duty_cycle);
  SerialUSB.println(ch6_duty_cycle);
  SerialUSB.println(ch7_duty_cycle);
  SerialUSB.println(" ");

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
    //head_servo_cmd = constrain(map(twosComp((packet[1] << 8) | packet[0]), -1000, 1000, 1000, 2000), 1000, 2000);

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
  analogWrite(left_wheel_speed_pin, abs(left_wheel_cmd));
  digitalWrite(right_wheel_dir_pin, (right_wheel_cmd > 0));
  analogWrite(right_wheel_speed_pin, abs(right_wheel_cmd));
  //head_servo.writeMicroseconds(head_servo_cmd);

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  //Send packet to computer
//  if (micros() - lastSend > sendPeriod) {
//    byte outGoingPacket[outGoingPacketLength];
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
//    //    outGoingPacket[14] = (byte)(ch8_duty_cycle & 0b0000000011111111);
//    //    outGoingPacket[15] = (byte)(ch8_duty_cycle >> 8);
//    //    outGoingPacket[16] = (byte)(ch9_duty_cycle & 0b0000000011111111);
//    //    outGoingPacket[17] = (byte)(ch9_duty_cycle >> 8);
//    //    outGoingPacket[18] = (byte)(ch10_duty_cycle & 0b0000000011111111);
//    //    outGoingPacket[19] = (byte)(ch10_duty_cycle >> 8);
//    //    outGoingPacket[20] = (byte)(ch11_duty_cycle & 0b0000000011111111);
//    //    outGoingPacket[21] = (byte)(ch11_duty_cycle >> 8);
//    outGoingPacket[24] = (byte)(left_wheel_pulses & 0b0000000011111111);
//    outGoingPacket[25] = (byte)(left_wheel_pulses >> 8 & 0b0000000011111111);
//    outGoingPacket[26] = (byte)(left_wheel_pulses >> 16 & 0b0000000011111111);
//    outGoingPacket[27] = (byte)(left_wheel_pulses >> 24);
//    outGoingPacket[28] = (byte)(right_wheel_pulses & 0b0000000011111111);
//    outGoingPacket[29] = (byte)(right_wheel_pulses >> 8 & 0b0000000011111111);
//    outGoingPacket[30] = (byte)(right_wheel_pulses >> 16 & 0b0000000011111111);
//    outGoingPacket[31] = (byte)(right_wheel_pulses >> 24);
//
//    FLOATUNION_t latitude;
//    latitude.number = 10.5;
//    FLOATUNION_t longitude;
//    longitude.number = 14.2;
//    FLOATUNION_t hdop; //horizontal dilution of precision for variance calculation and debugging
//    hdop.number = 1.7;
//
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
//
//    byte PEC = outGoingPacket[0];
//    for (int i = 1; i < outGoingPacketLength - 1; i++) {
//      PEC ^= outGoingPacket[i];
//    }
//    outGoingPacket[outGoingPacketLength - 1] = PEC;
//    Serial.write(outGoingPacket, outGoingPacketLength);
//
//    lastSend = micros();
//  }

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
