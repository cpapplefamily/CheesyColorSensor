#include <Arduino.h>

class ColorTrigger {
  private:
    int upperFilter[10];
    int runningTotal = 0;

  public:
    ColorTrigger();
    int putdata(int data);   
};
