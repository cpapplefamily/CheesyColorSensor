#include <Arduino.h>
#include "FastLED.h"


#define NUM_LEDS      10
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

#define SIGN_OF_LIFE_AR 0
#define SIGN_OF_LIFE_PI 1
#define UPPER_LED_START 2
#define LOWER_LED_START 6

boolean EN_CALIBRATE_PLOT = false;

enum SensorState {
                  NONE,
                  RED,
                  BLUE,
                  GREEN
                  };
//Eight storage location for the sensor States
SensorState upperSensorState[NUM_UPPER_SENSORS];
SensorState upperSensorScored_ONS[NUM_UPPER_SENSORS];
SensorState lowerSensorState[NUM_LOWER_SENSORS];
SensorState lowerSensorScored_ONS[NUM_LOWER_SENSORS];

//int upperFilter[NUM_UPPER_SENSORS][10];
//int lowerFilter[NUM_LOWER_SENSORS][10];

#include "ColorTrigger.h"

ColorTrigger upperTrigger[NUM_UPPER_SENSORS];
ColorTrigger lowerTrigger[NUM_LOWER_SENSORS];

#include "Debouncer.h"
double debounceTime = 60;
Debouncer::DebounceType debounceType = Debouncer::DebounceType::kBoth;

Debouncer upperDebounceRED[NUM_UPPER_SENSORS] = {
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}
                                             };
Debouncer upperDebounceBLUE[NUM_UPPER_SENSORS] = {
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}
                                             };
Debouncer lowerDebounceRED[NUM_LOWER_SENSORS] = {
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}
                                             };
Debouncer lowerDebounceBLUE[NUM_LOWER_SENSORS] = {
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}, 
                                             {debounceTime, debounceType}
                                             };

int reddata=0;        
int bluedata=0;   

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


//REturn the Sensor State and set the LED color CGRB byReference
SensorState getSensorState(GY_31 sensor, CRGB& led){
   SensorState state;

   //Get Sensor Color Reading
   reddata=map(sensor.getRED(),700,75,0,100);
   bluedata=map(sensor.getBLUE(),700,75,0,100);

   // the lower value for easier plotting. Value idiles @ -300
   if(reddata<-10){
      reddata = -10;
   }
   if(bluedata<-10){
      bluedata = -10;
   }

   //Set some Trigger Threasholds
   int redThresh = 50;
   int blueThresh = 50;
   if((reddata > redThresh) & (reddata > bluedata)){
      led = CRGB::Red;
      state = SensorState::RED;
   }else if((bluedata > blueThresh) & (bluedata > reddata)){
      led = CRGB::Blue;
      state = SensorState::BLUE;
   }else{
      led = CRGB::Black;
      state = SensorState::NONE;
   }
   if(EN_CALIBRATE_PLOT){
      Serial.print(bluedata); 
      Serial.print(","); 
      Serial.println(reddata);        
   }; 

   return state;
}


void setup() {
   //Open a serial port, currently for debugging but will be used for Arduino > RassperyPi > FMS data transfer
   Serial.begin(9600); 

   boolean Enable_All;
   if(EN_CALIBRATE_PLOT){
      Enable_All = false;
   }else{
      Enable_All = true;
   }
   //Set sensor state and turn on all LED's
   for(int i=0; i<NUM_UPPER_SENSORS ; i++){
      upperSensorState[i] = SensorState::NONE; 
      upperSensorScored_ONS[i] = SensorState::NONE; 
      upperTrigger[i].reset();
      upperSensors[i].enableLEDs(Enable_All);
   }   
      
    
   for(int i=0; i<NUM_LOWER_SENSORS ; i++){
      lowerSensorState[i] = SensorState::NONE; 
      lowerSensorScored_ONS[i] = SensorState::NONE; 
      lowerTrigger[i].reset();
      lowerSensors[i].enableLEDs(Enable_All);
   }   
     
    
   //Set up RGB LED strip lights
   FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
   FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
   FastLED.setBrightness(10);
}

void Setup_CALIBRATE_PLOT(int incomingByte){
   for(int i=0; i<NUM_LEDS; i++){
      leds[i] = CRGB::Black;
   }
   for(int i=0; i<NUM_UPPER_SENSORS ; i++){
         upperSensorState[i] = SensorState::NONE; 
         upperSensorScored_ONS[i] = SensorState::NONE; 
         upperTrigger[i].reset();
         upperSensors[i].enableLEDs(false);
   }
   for(int i=0; i<NUM_LOWER_SENSORS ; i++){
      lowerSensorState[i] = SensorState::NONE; 
      lowerSensorScored_ONS[i] = SensorState::NONE; 
      lowerTrigger[i].reset();
      lowerSensors[i].enableLEDs(false);
   }  
   if((incomingByte>=0) & (incomingByte<=3)){
      upperSensors[incomingByte].enableLEDs(true);
      Serial.println("Upper");
   }

   if((incomingByte>=4) & (incomingByte<=7)){
      Serial.println("Lower");
      lowerSensors[incomingByte - 4].enableLEDs(true);
   }
}


/**
 * Main Loop
 * 
 * 
 */

long int hartBeatTck = 0;
long int currentTime = 0;
boolean hartBeat = false;

/* goal_char_msg_map = {
    "S": '{ "type": "RU" }',
    "X": '{ "type": "RL" }',
    "Y": '{ "type": "BU" }',
    "H": '{ "type": "BL" }'
} */
int incomingByte = 0; // for incoming serial data
int int_Calibrate = 0; // for incoming serial data
bool signOfLifePi = false;
bool signOfLifePi_ONS = false;
bool signOfLifePi_ACTIVE = false;

void loop(){
   long int currentTime = millis();

   if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.read();
      if(((incomingByte) == '\r' || (incomingByte) == '\n')){
         //Do Nothing
      }else{
         //Convert to int
         int_Calibrate = incomingByte - '0';
         if(int_Calibrate == 9){
            EN_CALIBRATE_PLOT = true;
         }else if(int_Calibrate == 8){
            EN_CALIBRATE_PLOT = false;            
         }
      }
      if(EN_CALIBRATE_PLOT){
         Setup_CALIBRATE_PLOT(int_Calibrate);
      }
      signOfLifePi = true;
   }

   if(EN_CALIBRATE_PLOT){
      if((int_Calibrate>=0) & (int_Calibrate<=3)){
         upperSensorState[int_Calibrate] = getSensorState(upperSensors[int_Calibrate], leds[int_Calibrate + UPPER_LED_START]);
      }   
   }else{
      //Loop through upper sensors
      //for(int i=0; i < 1 ; i++){
      for(int i=0; i < NUM_UPPER_SENSORS ; i++){
         upperSensorState[i] = getSensorState(upperSensors[i], leds[i + UPPER_LED_START]);

         //Serial.println(upperSensorState[i]);
      
         if(upperDebounceRED[i].calculate((upperSensorState[i] == SensorState::RED))){
            //Serial.println("Is red");
            if((upperSensorScored_ONS[i]!= SensorState::RED)){
               //Serial.println("**********RED**************");
               Serial.print("S");
               upperSensorScored_ONS[i]= SensorState::RED;
               //delay(3000);
            }
         }else if(upperDebounceBLUE[i].calculate((upperSensorState[i] == SensorState::BLUE))){
            //Serial.println("Is blue");
            if((upperSensorScored_ONS[i]!= SensorState::BLUE)){
               //Serial.println("**********BLUE**************");
               Serial.print("Y");
               upperSensorScored_ONS[i]= SensorState::BLUE;
               //delay(3000);
            }
            
         }else{
            upperSensorScored_ONS[i]= SensorState::NONE;
         };   
      }
   }

   if(EN_CALIBRATE_PLOT){
      if((int_Calibrate>=4) & (int_Calibrate<=7)){
         lowerSensorState[int_Calibrate - 4] = getSensorState(lowerSensors[int_Calibrate - 4], leds[int_Calibrate - 4 + LOWER_LED_START]);
      }   
   }else{
      //Loop through lower sensors
      for(int i=0; i <NUM_LOWER_SENSORS ; i++){
         lowerSensorState[i] = getSensorState(lowerSensors[i], leds[i + LOWER_LED_START]);
         
         //Serial.println(lowerSensorState[i]);

         if(lowerDebounceRED[i].calculate((lowerSensorState[i] == SensorState::RED))){
            //Serial.println("Is red");
            if((lowerSensorScored_ONS[i]!= SensorState::RED)){
            // Serial.println("**********RED**************");
               Serial.print("X");
               lowerSensorScored_ONS[i]= SensorState::RED;
               //delay(3000);
            }
         }else if(lowerDebounceBLUE[i].calculate((lowerSensorState[i] == SensorState::BLUE))){
            //Serial.println("Is blue");
            if((lowerSensorScored_ONS[i]!= SensorState::BLUE)){
               //Serial.println("**********BLUE**************");
               Serial.print("H");
               lowerSensorScored_ONS[i]= SensorState::BLUE;
               //delay(3000);
            }
            
         }else{
            lowerSensorScored_ONS[i]= SensorState::NONE;
         };      
      }
   }
   //Flip Hartbeat LED
   if(currentTime > hartBeatTck){
      hartBeatTck = currentTime + 500;
      hartBeat = !hartBeat;
   }

   if(hartBeat){
      leds[SIGN_OF_LIFE_AR] = CRGB::White;
      if(signOfLifePi & !signOfLifePi_ONS){
         leds[SIGN_OF_LIFE_PI] = CRGB::Green;
         signOfLifePi_ACTIVE = true;
      }
      signOfLifePi_ONS = true;
   }else{
      leds[SIGN_OF_LIFE_AR] = CRGB::Black;
      if(signOfLifePi & signOfLifePi_ACTIVE){
         signOfLifePi = false;
         signOfLifePi_ONS = false;
         signOfLifePi_ACTIVE = false;
         leds[SIGN_OF_LIFE_PI] = CRGB::Black;
      }
   }

   FastLED.show();
   delay(20);
}
