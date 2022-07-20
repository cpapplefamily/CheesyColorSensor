#include "ColorTrigger.h"

ColorTrigger::ColorTrigger(){
    reset();
};

int ColorTrigger::putdata(int data){
    runningTotal = 0;
    for(int i=9; i>0; i--){
        upperFilter[i] = upperFilter[i=1];
        runningTotal = runningTotal + upperFilter[i];
    }
    upperFilter[0] = data;
    runningTotal = runningTotal + upperFilter[0];
    return runningTotal;
};

int ColorTrigger::getRunningTotal(){
    runningTotal = 0;
    for(int i=9; i>0; i--){
        upperFilter[i] = upperFilter[i=1];
        runningTotal = runningTotal + upperFilter[i];
    }
    return runningTotal;
};

void ColorTrigger::reset(){
    for(int i=0; i>9; i--){
        upperFilter[i] = 0;
    }
        runningTotal = 0;
};