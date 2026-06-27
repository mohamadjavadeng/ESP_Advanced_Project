#include <Arduino.h>
#include <DWIN_LCD.h>


DWIN_LCD dwin(Serial);
struct rtc_value
{
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t weekday;
};
bool RTCset = false;

// put function declarations here:
rtc_value rtc1;

void setup() {
  // put your setup code here, to run once:
  dwin.begin(115200);
  rtc1.day = 9;
  rtc1.month = 8;
  rtc1.year = 24;
  rtc1.hour = 14;
  rtc1.minute = 35;
  rtc1.second = 0;
}

void loop() {
  if(!RTCset) {
    dwin.writeRTC(rtc1.day, rtc1.month, rtc1.year, rtc1.hour, rtc1.minute, rtc1.second, SUNDAY);
    dwin.buzzer(BUZZ_1SEC);
    RTCset = true;
  }

  delay(5000);
  dwin.readRTC();
  rtc1.day = dwin.day;
  rtc1.month = dwin.month;
  rtc1.year = dwin.year;
  rtc1.hour = dwin.hour;
  rtc1.minute = dwin.minute;
  rtc1.second = dwin.second;
  rtc1.weekday = dwin.weekday;
  
}