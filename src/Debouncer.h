#include <Arduino.h>

class Debouncer {
  private:
    long int m_prevTimeSeconds;
    int m_debounceTimeSeconds;
    bool m_baseline;
    void resetTimer();
    boolean hasElapsed();

  public:
    enum DebounceType {kRising, kFalling, kBoth};
    DebounceType m_debounceType;
    Debouncer(double debounceTime, DebounceType type);
    //Debouncer(double debounceTime);
    boolean calculate(boolean input); 
};
