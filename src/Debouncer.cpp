#include "Debouncer.h"

/**
   * Creates a new Debouncer.
   *
   * @param debounceTime The number of seconds the value must change from baseline for the filtered
   *     value to change.
   * @param type Which type of state change the debouncing will be performed on.
   */
Debouncer::Debouncer(double debounceTime, DebounceType type){
    m_debounceTimeSeconds = debounceTime;
    m_debounceType = type;

    resetTimer();

    switch (m_debounceType) {
      case kBoth: // fall-through
      case kRising:
        m_baseline = false;
        break;
      case kFalling:
        m_baseline = true;
        break;
    }
};

/**
   * Creates a new Debouncer. Baseline value defaulted to "false."
   *
   * @param debounceTime The number of seconds the value must change from baseline for the filtered
   *     value to change.
   */
Debouncer::Debouncer(double debounceTime) {//Debouncer(double debounceTime);
    Debouncer(debounceTime, kRising);
};

void Debouncer::resetTimer(){
    m_prevTimeSeconds = millis();
};

boolean Debouncer::hasElapsed() {
    return millis() - m_prevTimeSeconds >= m_debounceTimeSeconds;
}

/**
* Applies the debouncer to the input stream.
*
* @param input The current value of the input stream.
* @return The debounced value of the input stream.
*/
boolean Debouncer::calculate(boolean input) {
    if (input == m_baseline) {
       resetTimer();
    }

    if (hasElapsed()) {
        if (m_debounceType == kBoth) {
          m_baseline = input;
          resetTimer();
        }
        return input;
    } else {
        return m_baseline;
    }
}