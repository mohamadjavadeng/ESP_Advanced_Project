#ifndef DWIN_h
#define DWIN_h

#include "Arduino.h"

/*
duraion of beeping
*/
typedef enum {
    BUZZ_1SEC = 0,
    BUZZ_500MSEC,
    BUZZ_250MSEC
}buzzer_duration;

/*
weekday of RTC
*/
typedef enum {
    SUNDAY = 0,
    MONDAY,
    TUESDAY,
    WEDNESDAY,
    THURSDAY,
    FRIDAY,
    SATURDAY
}weekdays;



class DWIN_LCD{
    public:
    DWIN_LCD(HardwareSerial& serial);

    uint8_t day, month, year, hour, minute, second, weekday;
    uint8_t backlightValue;
    uint8_t backlightCurrent;
    void begin(long int baudrate = 9600);
    bool isConnected(void);
    void nextPage(void);
    void previousPage(void);
    void gotoPage(const byte page);
    void writeSingleReg(const uint16_t registeraddress, const uint16_t value);
    void writeData(const uint16_t registeraddress, const uint8_t data[], const uint8_t length);
    void setSingleBit(const uint16_t registeraddress, const uint16_t Bitnumber);
    void resetSingleBit(const uint16_t registeraddress, const uint16_t Bitnumber);
    uint16_t readSingleReg(const uint16_t registeraddress);
    bool readSingleBit(const uint16_t registeraddress, const uint16_t Bitnumber);
    void readReg(const uint16_t registeraddress, byte numberOfBytes, uint8_t* data);
    void readRTC(void);
    void writeRTC(uint8_t dayrtc, uint8_t monthrtc, uint8_t yearrtc, uint8_t hourrtc, uint8_t minutertc, uint8_t secondrtc, weekdays weekdayrtc);
    void backlight(void);
    void buzzer(buzzer_duration buzzer= BUZZ_1SEC);

    /*void startAnimation(uint16_t VP);
    void stopAnimation(uint16_t VP);*/


    /*
    touch panel sound on/off
    set backlight
    rotating background
    */

    private:
    HardwareSerial& _serial;
    uint8_t _response[30];
    bool _readResponse(void);
};






#endif