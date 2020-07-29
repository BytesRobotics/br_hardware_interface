/*
  This library is for the Tube Safety System (TSS) by Bytes Robotics.
  Created by Gavin Remme and Micheal Equi, 6/12/20
  Based around the MS5607-02BA03 barometric pressure sensor: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5607-02BA03&DocType=Data+Sheet&DocLang=English
*/
#ifndef TSS_h
#define TSS_h
#ifndef Wire_h
#define Wire_h

#include "Arduino.h"
#include <Wire.h>

//definitions of values used to talk to barometric pressure sensors
#define MS5xxx_CMD_RESET    0x1E    // perform reset
#define MS5xxx_CMD_ADC_READ 0x00    // initiate read sequence
#define MS5xxx_CMD_ADC_CONV 0x40    // start conversion    
#define MS5xxx_CMD_ADC_D1   0x00    // read ADC 1
#define MS5xxx_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256
#define SMART_ARRAY_SIZE 2000    //Defines the smart array size. Temporary implementation
//unsigned long value_l = 0, value_r = 0; //global variables (ew) for the raw baro values

//Variables beginning with an underscore are private variables (e.g. _index is the private version of index)
class TSS
{
  public:
    TSS(TwoWire* wire, uint8_t addr);
    long lastDebounceTime = 0; //stores when the debounce begins
  private:
    TwoWire* _wire;
    uint8_t _addr;
    int _impactThreshold; //threshold above average sensors value that must be exceeded to count as an impact
    int _releaseThreshold; //threshold below average sensor value that must be crossed to count as a release
    unsigned long sensor_read_start_time = 0;  //values to hold system time that sensors began reading
    bool reading = false; //bools to track when sensors are reading
    unsigned long read_delay = 1000; //time to wait for the sensor to take reading before asking for data
    unsigned long value = 0; //raw values coming off sensors
    long debounceDelay = 150000; //debounce time to wait for pressure to fall below impact threshold. Units of microseconds
    bool impact = false; //bool to hold if sensor is impacted
  public:
    void setImpactThreshold(int x); //sets impact threshold, recommended value of 3000
    void setReleaseThreshold(int x); //sets release threshold, recommeded value of 3000
    void send_cmd(byte aCMD); //sends command to sensors
    bool sensor_connect(); //requests to connect to sensor at a specific address (0x76 or 0x77)
    void read_sensor_value(unsigned long& value); 
    bool init_sensor(); //initialize sensor, include I2C address in brackets
    bool newval(); //poll the sensor for a new value, only requests if enough time has passed to do so
    unsigned long filter_val(); //returns lightly filtered sensor value
    unsigned long get_average(); //returns longer, running average from SmartArray
    unsigned long impact_Threshold(); //returns value the sensor must exceed for an impact even to be triggered
    unsigned long release_Threshold(); //returns value the sensor must fall below for release event to trigger
    bool _impact(); //returns T/F as to if an impact has occured. Pulls a new value from the sensor as well; contains newval()
    bool _release(); //same functionality as _impact(), except pertains to release threshold
};
#endif
#endif
