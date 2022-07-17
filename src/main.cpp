#include <Arduino.h>
#include "FastLED.h"


#define NUM_LEDS      9
#define LED_TYPE   WS2812B
#define COLOR_ORDER   GRB
#define DATA_PIN        3
//#define CLK_PIN       4
#define VOLTS          5
#define MAX_MA       1000

//Sensor 1
#define led_EN_1 8
#define s2_1 10
#define s3_1 9
#define out_1 11

//Sensor 2
#define led_EN_2 4
#define s2_2 6
#define s3_2 5
#define out_2 7

//Sensor 3
#define led_EN_3 17
#define s2_3 15
#define s3_3 16
#define out_3 14

//Sensor 4
#define led_EN_4 21
#define s2_4 19
#define s3_4 20
#define out_4 18

//Sensor 5
#define led_EN_5 29
#define s2_5 27
#define s3_5 28
#define out_5 26

//Sensor 6
#define led_EN_6 33
#define s2_6 31
#define s3_6 32
#define out_6 30

//Sensor 7
#define led_EN_7 37
#define s2_7 35
#define s3_7 36
#define out_7 34

//Sensor 8
#define led_EN_8 41
#define s2_8 39
#define s3_8 40
#define out_8 38

#define NUM_UPPER_SENSORS 4
#define NUM_LOWER_SENSORS 4

enum SensorState {
                  NONE,
                  RED,
                  BLUE,
                  GREEN
                  };
//Eight storage location for the sensor States
SensorState upperSensorState[NUM_UPPER_SENSORS];
SensorState _upperSensorState[NUM_UPPER_SENSORS];
SensorState lowerSensorState[NUM_LOWER_SENSORS];
SensorState _lowerSensorState[NUM_LOWER_SENSORS];

int reddata=0;        
int bluedata=0;   
bool EN_PLOT = false;     

#include "GY_31.h"

GY_31 upperSensors[] = {  
                     {s2_1, s3_1, out_1, led_EN_1}, 
                     {s2_2, s3_2, out_2, led_EN_2}, 
                     {s2_3, s3_3, out_3, led_EN_3}, 
                     {s2_4, s3_4, out_4, led_EN_4}};
GY_31 lowerSensors[] = { 
                     {s2_5, s3_5, out_5, led_EN_5}, 
                     {s2_6, s3_6, out_6, led_EN_6}, 
                     {s2_7, s3_7, out_7, led_EN_7}, 
                     {s2_8, s3_8, out_8, led_EN_8}};

CRGBArray<NUM_LEDS> leds;


/* void TestSensor1(){
   long int t1 = millis();
   
   reddata=sensors[0].getRED();
   bluedata=sensors[0].getBLUE();
   if(!EN_PLOT){
      Serial.print("Red 1 value= "); 
      Serial.print(map(reddata,700,75,0,100));        
   }else{
      //Serial.print("Red: "); 
      Serial.print(map(bluedata,700,75,0,100)); 
      Serial.print(","); 

      Serial.println(map(reddata,700,75,0,100));        
   };
   if(!EN_PLOT){
      Serial.print("\t"); 
      long int t2 = millis();
      Serial.println("Time taken by the task: "); 
      Serial.print(t2-t1); Serial.println(" milliseconds");
      Serial.println();   
   }
   delay(200);
}

void marqueLED(){
   unsigned long stepdelay = 1000;

   sensors[0].enableLEDs();
   delay(stepdelay);
   sensors[0].disableLEDs();
   sensors[1].enableLEDs();
   delay(stepdelay);
   sensors[1].disableLEDs();
   sensors[2].enableLEDs();
   delay(stepdelay);
   sensors[2].disableLEDs();
   sensors[3].enableLEDs();
   delay(stepdelay);
   sensors[3].disableLEDs();
   sensors[4].enableLEDs();
   delay(stepdelay);
   sensors[4].disableLEDs();
   sensors[5].enableLEDs();
   delay(stepdelay);
   sensors[5].disableLEDs();
   sensors[6].enableLEDs();
   delay(stepdelay);
   sensors[6].disableLEDs();
   sensors[7].enableLEDs();
   delay(stepdelay);
   sensors[7].disableLEDs();
}

void ScanValues(){
   
   long int t1 = millis();
   
   Serial.print("Red 1 value= "); 
   reddata=sensors[0].getRED();
   Serial.print(map(reddata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 1 value= ");
   reddata=sensors[0].getBLUE();
   Serial.print(map(reddata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
   
   Serial.print("Red 2 value= "); 
   reddata=sensors[1].getRED();
   Serial.print(map(reddata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 2 value= ");
   reddata=sensors[1].getBLUE();
   Serial.print(map(reddata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
       


   long int t2 = millis();
   Serial.println("Time taken by the task: "); 
   Serial.print(t2-t1); Serial.println(" milliseconds");
   Serial.println();

   delay(200);
} */

//REturn the Sensor State and set the LED color CGRB byReference
SensorState testSensor(GY_31 sensor, CRGB& led){
   SensorState state;
   reddata=map(sensor.getRED(),700,75,0,100);
   bluedata=map(sensor.getBLUE(),700,75,0,100);
   if(reddata<-10){
      reddata = -10;
   }
   if(bluedata<-10){
      bluedata = -10;
   }
   int redThresh = 50;
   int blueThresh = 50;
   if((reddata>redThresh) & (reddata>bluedata)){
      led = CRGB::Red;
      state = SensorState::RED;
   }else if((bluedata>blueThresh) & (bluedata>reddata)){
      led = CRGB::Blue;
      state = SensorState::BLUE;
   }else{
      led = CRGB::Black;
      state = SensorState::NONE;
   }
/*    if(!EN_PLOT){
      Serial.print("Red value= "); 
      Serial.print(reddata);   
      Serial.print("\t");   
      Serial.print("Blue value= "); 
      Serial.println(bluedata);      
   }else{
      Serial.print(bluedata); 
      Serial.print(","); 
      Serial.println(reddata);        
   }; */

   return state;
}


void setup() {
   Serial.begin(9600); 

   for(int i=0; i<NUM_UPPER_SENSORS ; i++){
      upperSensorState[i] = SensorState::NONE; 
      _upperSensorState[i] = SensorState::NONE; 
      upperSensors[i].enableLEDs(true);
   }   
   for(int i=0; i<NUM_LOWER_SENSORS ; i++){
      lowerSensorState[i] = SensorState::NONE; 
      _lowerSensorState[i] = SensorState::NONE; 
      lowerSensors[i].enableLEDs(true);
   }   
    
   
   FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
   FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
   FastLED.setBrightness(10);
}

long int hartBeatTck = 0;
long int currentTime = 0;
boolean hartBeat = false;

void loop(){
   long int currentTime = millis();

   for(int i=0; i<NUM_UPPER_SENSORS ; i++){
      upperSensorState[i] = testSensor(upperSensors[i], leds[i+1]);
      if(_upperSensorState[i]!=upperSensorState[i]){
         if(!EN_PLOT){
            Serial.print("Senesor-" + String(i) + " ");
            Serial.println(upperSensorState[i]);
         }
         _upperSensorState[i]=upperSensorState[i];
      } 
   }

   int lowerLED_Offset = 5;

   for(int i=0; i<NUM_LOWER_SENSORS ; i++){
      lowerSensorState[i] = testSensor(lowerSensors[i], leds[i+lowerLED_Offset]);
      if(_lowerSensorState[i]!=lowerSensorState[i]){
         if(!EN_PLOT){
            Serial.print("Senesor-" + String(i) + " ");
            Serial.println(lowerSensorState[i]);
         }
         _lowerSensorState[i]=lowerSensorState[i];
      } 
   }

   //Flip Hartbeat LED
   if(currentTime > hartBeatTck){
      hartBeatTck = currentTime + 500;
      hartBeat = !hartBeat;
   }
   if(hartBeat){
      leds[0] = CRGB::White;
   }else{
      leds[0] = CRGB::Black;
   }

   FastLED.show();
   delay(20);
}
