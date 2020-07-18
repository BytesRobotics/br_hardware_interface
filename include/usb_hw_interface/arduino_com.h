#ifndef ARDUINO_COM_H
#define ARDUINO_COM_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdint>
#include <bitset>
#include <cmath>

#include "serial/serial.h"

/**
* @breif constrians value between min and max inclusive. Value is returned by reference.
* @param[in,out] value input to be constrianed
* @param[in] min The minimum value that "value" should be able to be
* @param[in] max The maximum value that "value" should be able to be
*/
template <class T>
void constrain(T &value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
}

/**
* @breif returns a number mapped proportioanlly from one range of numbers to another
* @param[in] input Value to be mapped
* @param[in] inMax The maximum value for the range of the input
* @param[in] inMin The minimum value for the range of the input
* @param[in] outMin The minimum value for the range of the output
* @param[in] outMax The maximum value for the range of the output
* @return The input trnslated proportionally from range in to range out
*/
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}


/// Distance sensor enumerations
enum class DistSensor { front, front_left, front_right, front_left_bottom, front_right_bottom,
    left, right, rear, rear_left, rear_right, rear_left_bottom, rear_right_bottom};

/// TSS enumerations
enum class TSS {front_left, front_right, rear_left, rear_right};

typedef union
{
    float number;
    uint8_t bytes[4];
} FLOATUNION_t;


class HardwareCom
{
  ///Serial object that manages USB serial connection_
  serial::Serial connection_;

  // https://bytesrobotics.atlassian.net/wiki/spaces/BRO/pages/44826625/Hardware+Interface+Node
  const int outgoing_packet_length_ = 7;
  uint8_t outgoing_packet_[7]{};   //!Packet to the arduino (length = 6 + PEC)

  const int incoming_packet_length_ = 55;
  uint8_t incoming_packet_[55]{}; //!Packet received from the hardware controller

  const int num_channels_{12};
  int channels_[12] = {}; //!Values from CH1 to Ch5 for distance sensors
  int last_channels_[12] = {}; //!Values from CH1 to Ch5 for distance sensor filtering
  bool channels_last_update_skipped_[12] = {}; //!Did we skip the last value because of a max_diff_ problem
  const int max_diff_{3000}; //!The maximum difference between values for it to be accepted. Above this we assume the value is bad

  //  front_dist_pin  CH1
  //  left_dist_pin   CH2
  //  right_dist_pin  CH3
  //  rear_dist_pin   CH4
  //  bottom_dist_pin CH5

  long encoder_left_{}, encoder_right_{};
  FLOATUNION_t latitude_{}, longitude_{}, hdop_{};
  float speed_{}, angle_{}, altitude_{}; //!ground speed_, angle_ from north, and altitude_ from sea level
  int fix_{}, fix_quality_{}, satellites_{};

  uint8_t tss_byte_;

public:
  HardwareCom(const std::string& port, int baud);

  bool set_controller(double rmotor_cmd, double lmotor_cmd, double head_cmd); //!Sends input commands to the hardware controller
  bool read_controller(); //!Stores data read from the hardware controller

  /// Data accessor methods
  int get_dist_sensor_value(DistSensor sensor);
  long get_left_encoder() const;
  long get_right_encoder() const;
  float get_latitude() const;
  float get_longitude() const;
  float get_hdop() const;
  float get_speed() const;
  float get_angle() const;
  float get_altitude() const;
  int get_fix() const;
  int get_fix_quality() const;
  int get_num_satellites() const;
  bool tss_has_collision(TSS section) const;
};

#endif
