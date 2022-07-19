#include "ColorTrigger.h"

ColorTrigger::ColorTrigger(){
    
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