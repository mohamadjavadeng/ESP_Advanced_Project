#include <Arduino.h>
#include <DWIN_LCD.h>


DWIN_LCD dwin(Serial);

void setup() {
  // put your setup code here, to run once:
  dwin.begin(115200);
}

void loop() {
  uint16_t address = 0x2000;
  uint8_t datawrite[] = {'H','e','l','l','o'}; 
  dwin.writeData(address, datawrite, sizeof(datawrite));
  delay(2000);
  uint8_t datawrite1[] = {'M','o','h','a','m','m','a','d','j'};
  dwin.writeData(address, datawrite1, sizeof(datawrite1));
  delay(2000);
  dwin.writeSingleReg(0x2010, 452);
  delay(2000);
  uint16_t addressbit = 0x0010;
  dwin.setSingleBit(addressbit, 5); // 0b0000000000100000
}