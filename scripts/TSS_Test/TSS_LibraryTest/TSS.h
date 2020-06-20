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
    void setImpactThreshold(int x);
    void setReleaseThreshold(int x);
    void send_cmd(char addr, byte aCMD);
    bool sensor_connect(char addr);
    void read_sensor_value(char addr, unsigned long& value);
    bool init_sensor(char addr); //initialize sensor, include I2C address in brackets
    bool baro_read(); //reads both sensors in rapid succession
    bool right_newval();
    bool left_newval();
    unsigned long filter_rightval();
    unsigned long filter_leftval();
    unsigned long right_getaverage();
    unsigned long left_getaverage();
    unsigned long rightimpactThreshold();
    unsigned long leftimpactThreshold();
    unsigned long rightreleaseThreshold();
    unsigned long leftreleaseThreshold();
    bool r_impact();
    bool l_impact();
    long rightlastDebounceTime = 0; //stores when the debounce begins, doubles as exact time impact occured
    long leftlastDebounceTime = 0;
  private:
    int _impactThreshold; //threshold above average sensors value that must be exceeded to count as an impact
    int _releaseThreshold; //threshold below average sensor value that must be crossed to count as a release
    unsigned long sensor_read_start_time = 0, left_sensor_read_start_time = 0, right_sensor_read_start_time = 0;  //values to hold system time that sensors began reading
    bool reading = false, left_sensor_is_reading = false, right_sensor_is_reading = false; //bools to track when sensors are reading
    unsigned long read_delay = 1000; //time to wait for the sensor to take reading before asking for data
    unsigned long value_l = 0, value_r = 0; //raw values coming off sensors
    unsigned long rightImpactTime; //system time at which the sensors are impacted
    unsigned long leftImpactTime;
    long debounceDelay = 150000; //debounce time to wait for pressure to fall below impact threshold. Units of microseconds
    bool impact = false; //not currently used
};
#endif
#endif
