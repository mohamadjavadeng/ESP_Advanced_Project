#include <Arduino.h>
#include <DWIN_LCD.h>


DWIN_LCD dwin(Serial);

void setup() {
  // put your setup code here, to run once:
  dwin.begin(115200);
}

void loop() {
  byte page;
  page = 7;
  for(int i=0; i < 10; i++){
    dwin.nextPage();
    delay(1000);
  }
  
  for(int i = 0; i < 5; i++){
    dwin.previousPage();
    delay(1000);
  }
  dwin.gotoPage(page);
  delay(1000);
}