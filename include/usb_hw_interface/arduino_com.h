#ifndef ARDUINO_COM_H
#define ARDUINO_COM_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdint>
#include <bitset>

#include "serial/serial.h"

typedef union
{
    float number;
    uint8_t bytes[4];
} FLOATUNION_t;


class HardwareCom
{
  //Serial object that manages USB serial connection
  serial::Serial connection;
  //Packet structure = |PEC|lwheel MSB|lwheel LSB|rwheel MSB|rwheel LSB|head MSB|head LSB
  const int outgoingPacketLength = 7;
  uint8_t outgoingPacket[7];   //Packet to the arduino (length = 6 + PEC)
  //Packet structure = //Outgoing packet structure: |ch1 LSB|ch1 MSB|ch2 LSB|ch2 MSB|ch3 LSB|ch3 MSB|ch4 LSB|ch4 MSB|ch5 LSB|ch5 MSB|
    // |encoder_left LSB|encoder_left|encoder_left|encoder_left MSB|
    // |encoder_right LSB|encoder_right|encoder_right|encoder_right MSB|
    // |lat LSB|lat|lat|lat MSB|
    // |lon LSB|lon|lon|lon MSB|
    // |speed LSB|speed MSB|angle LSB|angle MSB|altitude LSB|altitude MSB|
    // |fix|fix quality|num satellites|
    // |PEC|
  const int incomingPacketLength = 36;
  uint8_t incomingPacket[36]; //Packet from the zero (length = 12 + PEC)
  int channels[5] = {}; //Values from CH1 to Ch5 for distance sensors
  //  front_dist_pin  CH1
  //  left_dist_pin   CH2
  //  right_dist_pin  CH3
  //  rear_dist_pin   CH4
  //  bottom_dist_pin CH5

  long encoderLeft, encoderRight;
  FLOATUNION_t latitude, longitude;
  float speed, angle, altitude; //ground speed, angle from north, and altitude from sea level
  int fix, fix_quality, satellites;



public:
  HardwareCom(std::string port, int baud);            //Constructor
  bool setController(double rmotor_cmd, double lmotor_cmd, double head_cmd); //Sends input commands to arduino zero
  bool readController();                            //Returns data read from the arduino zero
  int getCh1();
  int getCh2();
  int getCh3();
  int getCh4();
  int getCh5();
  long getEncoderLeft();
  long getEncoderRight();
  float getLatitude();
  float getLongitude();
  float getSpeed();
  float getAngle();
  float getAltitude();
  int getFix();
  int getFixQuality();
  int getNumSatellites();
};

#endif
