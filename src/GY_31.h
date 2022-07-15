#include <Arduino.h>

class GY_31 {
  private:
    int S2_Pin;
    int S3_Pin;
    int OUT_Pin;
    int LED_Pin;
    int dat = 0;

  public:
    GY_31(int S2pin, int S3pin, int OUTpin, int ENled);
    int getRED();
    int getBLUE();
    void disableLEDs();
    void enableLEDs();
    void enableLEDs(boolean enable);
    
};
