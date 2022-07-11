// LED.h
#ifndef LED_h
#define LED_h

#include <Arduino.h>

class LED {
  private:
    int ledPin;
    unsigned char ledState;

  public:
    LED(int pin);
    void turnON();
    void turnOFF();
    int getState();
};

#endif