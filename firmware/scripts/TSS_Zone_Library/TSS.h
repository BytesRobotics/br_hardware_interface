/*
  This library is for the Tube Safety System (TSS) by Bytes Robotics.
  Created by Gavin Remme and Micheal Equi, 6/12/20
  Based around the MS5607-02BA03 barometric pressure sensor: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5607-02BA03&DocType=Data+Sheet&DocLang=English
*/
#ifndef TSS_H
#define TSS_H

#include "Arduino.h"
#include "SmartArray.h"
#include <Wire.h>

//definitions of values used to talk to barometric pressure sensors
#define MS5xxx_CMD_RESET    0x1E    // perform reset
#define MS5xxx_CMD_ADC_READ 0x00    // initiate read sequence
#define MS5xxx_CMD_ADC_CONV 0x40    // start conversion    
#define MS5xxx_CMD_ADC_D1   0x00    // read ADC 1
#define MS5xxx_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256

//Variables beginning with an underscore are private variables (e.g. _index is the private version of index)
class TSS
{
    // Wire variables
    TwoWire* _wire;
    char _addr;
    
    // Impact State Variables
    const unsigned long _read_delay = 1000; //time to wait for the sensor to take reading before asking for data
    const unsigned long _debounce_delay = 150000; //debounce time to wait for pressure to fall below impact threshold. Units of microseconds

    int _impact_threshold = 0; //threshold above average sensors value that must be exceeded to count as an impact
    int _release_threshold = 0; //threshold below average sensor value that must be crossed to count as a release
    unsigned long _sensor_read_start_time = 0;  //values to hold system time that sensors began reading
    long _last_debounce_time = 0; //stores when the debounce begins
    bool _impact = false; //bool to hold if sensor is impacted
    bool _reading = false;

    inline bool is_impacted(); //returns T/F as to if an impact has occured. Pulls a new value from the sensor as well; contains newval()
    inline bool is_released(); //same functionality as _impact(), except pertains to release threshold

    inline void send_cmd(byte aCMD); //sends command to sensors
    inline bool init_sensor();    //initialize sensor, include I2C address in brackets
    inline bool sensor_connect(); //requests to connect to sensor at a specific address (0x76 or 0x77)
    inline void read_sensor_value(unsigned long& value);

    SmartArray _array; //instance of the SmartArray to handle the sensor values

  public:
    TSS(TwoWire* wire, char addr);

    void set_impact_threshold(int x); //sets impact threshold, recommended value of 3000
    void set_release_threshold(int x); //sets release threshold, recommeded value of 3000
    bool get_new_val(); //poll the sensor for a new value, only requests if enough time has passed to do so

    unsigned long get_filter_val();        //returns lightly filtered sensor value
    unsigned long get_average();           //returns longer, running average from SmartArray
    unsigned long get_impact_threshold();  //returns value the sensor must exceed for an impact even to be triggered
    unsigned long get_release_threshold(); //returns value the sensor must fall below for release event to trigger

};
#endif
