#include <Arduino.h>
#include "FastLED.h"
//#include <Servo.h>
#include <Adafruit_TiCoServo.h>


//Servo myservo;  // create servo object to control a servo
Adafruit_TiCoServo myservo;  // create servo object to control a servo

#define PWM_PIN        2
int potpin = A0;  // analog pin used to connect the potentiometer
int val;

//LEDs start at count 0
#define NUM_LEDS      202
#define NUM_UPPER_BLOCK_START 2
#define NUM_UPPER_BLOCK_LEN   45
#define NUM_LOWER_BLOCK_START 191
#define NUM_LOWER_BLOCK_LEN   1

#define LED_TYPE   WS2812B
#define COLOR_ORDER   GRB
#define DATA_PIN        3
//#define CLK_PIN       4
#define VOLTS           5
#define MAX_MA       1000

//Sensor 1
#define led_EN_1  8
#define s2_1      10
#define s3_1      9
#define out_1     11

//Sensor 2
#define led_EN_2  4
#define s2_2      6
#define s3_2      5
#define out_2     7

//Sensor 3
#define led_EN_3  17
#define s2_3      15
#define s3_3      16
#define out_3     14

//Sensor 4
#define led_EN_4  21
#define s2_4      19
#define s3_4      20
#define out_4     18

//Sensor 5
#define led_EN_5  29
#define s2_5      27
#define s3_5      28
#define out_5     26

//Sensor 6
#define led_EN_6  33
#define s2_6      31
#define s3_6      32
#define out_6     30

//Sensor 7
#define led_EN_7  37
#define s2_7      35
#define s3_7      36
#define out_7     34

//Sensor 8
#define led_EN_8  41
#define s2_8      39
#define s3_8      40
#define out_8     38

#define NUM_UPPER_SENSORS 4
#define NUM_LOWER_SENSORS 4

#define LOWER_SCALE_LIM 0
#define UPPER_SCALE_LIM 100

#define SIGN_OF_LIFE_AR 0
#define SIGN_OF_LIFE_PI 1


boolean EN_CALIBRATE_PLOT = false;


enum SensorState {
                  NONE,
                  RED,
                  BLUE,
                  GREEN
                  };

enum MatchState {
               PreMatch,         // 20
               StartMatch,       // 21
               WarmupPeriod,     // 22
               AutoPeriod,       // 23
               PausePeriod,      // 24
               TeleopPeriod,     // 25
               PostMatch,        // 26
               TimeoutActive,    // 27
               PostTimeout,      // 28
               None              // 29
               };

CRGB MatchState_LEDs = CRGB::Black;
int matchState_int; //Not set

//Eight storage location for the sensor States
SensorState upperSensorState[NUM_UPPER_SENSORS];
SensorState upperSensorScored_ONS[NUM_UPPER_SENSORS];
SensorState lowerSensorState[NUM_LOWER_SENSORS];
SensorState lowerSensorScored_ONS[NUM_LOWER_SENSORS];

#include "ColorTrigger.h"

ColorTrigger upperTrigger[NUM_UPPER_SENSORS];
ColorTrigger lowerTrigger[NUM_LOWER_SENSORS];

#include "Debouncer.h"
double debounceTime_RED = 60;  //Sensor must be on or off
double debounceTime_BLUE = 60;  //Sensor must be on or off
Debouncer::DebounceType debounceType = Debouncer::DebounceType::kBoth;

Debouncer upperDebounceRED[NUM_UPPER_SENSORS] = {
                                             {debounceTime_RED, debounceType}, 
                                             {debounceTime_RED, debounceType}, 
                                             {debounceTime_RED, debounceType}, 
                                             {debounceTime_RED, debounceType}
                                             };
Debouncer upperDebounceBLUE[NUM_UPPER_SENSORS] = {
                                             {debounceTime_BLUE, debounceType}, 
                                             {debounceTime_BLUE, debounceType}, 
                                             {debounceTime_BLUE, debounceType}, 
                                             {debounceTime_BLUE, debounceType}
                                             };
Debouncer lowerDebounceRED[NUM_LOWER_SENSORS] = {
                                             {debounceTime_RED, debounceType}, 
                                             {debounceTime_RED, debounceType}, 
                                             {debounceTime_RED, debounceType}, 
                                             {debounceTime_RED, debounceType}
                                             };
Debouncer lowerDebounceBLUE[NUM_LOWER_SENSORS] = {
                                             {debounceTime_BLUE, debounceType}, 
                                             {debounceTime_BLUE, debounceType}, 
                                             {debounceTime_BLUE, debounceType}, 
                                             {debounceTime_BLUE, debounceType}
                                             };

//**********************//
// Set Up Color sensors //
//**********************//
int reddata=0;        
int bluedata=0;   

#include "GY_31.h"
unsigned long timeout_micros = 5000;// 2000 = 2.00ms//unsigned long = 1000000UL

GY_31 upperSensors[] = {  
                     {s2_1, s3_1, out_1, led_EN_1, timeout_micros}, 
                     {s2_2, s3_2, out_2, led_EN_2, timeout_micros}, 
                     {s2_3, s3_3, out_3, led_EN_3, timeout_micros}, 
                     {s2_4, s3_4, out_4, led_EN_4, timeout_micros}};
GY_31 lowerSensors[] = { 
                     {s2_5, s3_5, out_5, led_EN_5, timeout_micros}, 
                     {s2_6, s3_6, out_6, led_EN_6, timeout_micros}, 
                     {s2_7, s3_7, out_7, led_EN_7, timeout_micros}, 
                     {s2_8, s3_8, out_8, led_EN_8, timeout_micros}};

//**********************//
// Set Up led strip     //
//**********************//
CRGBArray<NUM_LEDS> leds;


//REturn the Sensor State and set the LED color CGRB byReference
//SensorState getSensorState(GY_31 sensor, CRGB& led){
 
bool plot_raw_data = false;
//REturn the Sensor State
SensorState getSensorState(GY_31 sensor){
   SensorState state;
   
   //Get Sensor Color Reading
   if(plot_raw_data){
      reddata=sensor.getRED();
      bluedata=sensor.getBLUE();
   }else{
      reddata=map(sensor.getRED(),700,75,LOWER_SCALE_LIM,UPPER_SCALE_LIM);
      bluedata=map(sensor.getBLUE(),700,75,LOWER_SCALE_LIM,UPPER_SCALE_LIM);
      // the lower value for easier plotting. Value idiles @ -300
      // valus above Upper scale limit indicates no sensor
      if(reddata<-10 | reddata > UPPER_SCALE_LIM){
         //reddata = -10;
      }
      if(bluedata<-10 | bluedata > UPPER_SCALE_LIM){
       //bluedata = -10;
      }
   }
   
   //Set some Trigger Threasholds
   int redThresh = 95;
   int blueThresh = 90;
   if((reddata > redThresh) & (reddata > bluedata)){
      state = SensorState::RED;
   }else if((bluedata > blueThresh) & (bluedata > reddata)){
      state = SensorState::BLUE;
   }else{
      state = SensorState::NONE;
   }
   if(EN_CALIBRATE_PLOT){
      Serial.print(bluedata); 
      Serial.print(","); 
      Serial.println(reddata);        
   }; 

   return state;
}

void configerSensorLED(boolean Enable_All){
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
}
void setup() {
   //Open a serial port, currently for debugging but will be used for Arduino > RassperyPi > FMS data transfer
   Serial.begin(9600); 
   // Not sure if this is needed anymore. Had something to do with Serial.Read() or maybe is needed for Serial.parseInt()
   Serial.setTimeout(20);

   myservo.attach(PWM_PIN);  // attaches the servo on PWM_PIN to the servo object

   boolean Enable_All;
   if(EN_CALIBRATE_PLOT){
      Enable_All = false;
   }else{
      Enable_All = true;
   }
   //Set sensor state and turn on all LED's
   configerSensorLED(Enable_All);

   matchState_int = 99;
    
   //Set up RGB LED strip lights
   FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
   FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
   FastLED.setBrightness(10);
}

void Setup_CALIBRATE_PLOT(int incomingByte){
   for(int i=0; i<NUM_LEDS; i++){
      leds[i] = CRGB::Black;
   }

   configerSensorLED(false); 

   if((incomingByte>=0) & (incomingByte<=3)){
      upperSensors[incomingByte].enableLEDs(true);
      Serial.println("Upper");
   }

   if((incomingByte>=4) & (incomingByte<=7)){
      Serial.println("Lower");
      lowerSensors[incomingByte - 4].enableLEDs(true);
   }
}

void fill_Block(int fill, int block, CRGB color){
   for(int l=fill; l < (fill+block); l++){
      leds[l] = color;
    }
}

CRGB setMatchStateLED(int matchState){
   switch (matchState){
      case 20: //PreMatch
         return CRGB::Green;
         break;
      case 21 ... 25: //StartMatch
         return CRGB::Black;
         break;
      case 26: // PostMatch
         return CRGB::Violet;
         break;
      case 27 ... 28: // TimeoutActive
         return CRGB::Black;
         break;
      default:
         return CRGB::Black;
         break;
   }
}

void run_Agitator(boolean run){
   if(run){
      val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
      val = map(val, 0, 1023, 45, 135);     // scale it for use with the servo (value between 0 and 180)
   }else{
      val = 90;
   }
   myservo.write(val);   
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
bool signOfLifePi_State = false;

void loop(){
   long int currentTime = millis();

   if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.parseInt();
      
      signOfLifePi = true;
   
      if(((incomingByte) == '\r' || (incomingByte) == '\n')){
         //Do Nothing
      }else{
         //store incomingByte
         int_Calibrate = incomingByte;
         if(int_Calibrate == 9){
            EN_CALIBRATE_PLOT = true;
         }else if(int_Calibrate == 8){
            EN_CALIBRATE_PLOT = false; 
            configerSensorLED(true);           
         }
         if((incomingByte) >= 20 & (incomingByte <= 29)){
            matchState_int = incomingByte;
         }

      }
      if(EN_CALIBRATE_PLOT){
         Setup_CALIBRATE_PLOT(int_Calibrate);
      }
   }

   //set the led backgound color
   MatchState_LEDs = setMatchStateLED(matchState_int);

   // Run Agitator if match running
   if(matchState_int>20 & matchState_int<26){
         run_Agitator(true);
   }else{
      run_Agitator(false);
   }

   if(EN_CALIBRATE_PLOT){
      if((int_Calibrate>=0) & (int_Calibrate<=3)){
         upperSensorState[int_Calibrate] = getSensorState(upperSensors[int_Calibrate]);
      }   
   }else{
      //Loop through upper sensors
      for(int i=0; i < NUM_UPPER_SENSORS ; i++){

         long int startsence = micros();  //For Time diagnostics

         upperSensorState[i] = getSensorState(upperSensors[i]);

         boolean printSensor_time = false;
         if((i == 3) & (printSensor_time)){ 
            Serial.print("Sensor 3 ");
            Serial.print(micros() - startsence);
            Serial.println(" us");
         }
      
         if(upperDebounceRED[i].calculate((upperSensorState[i] == SensorState::RED))){
            //Serial.println("Is red");
            if((upperSensorScored_ONS[i]!= SensorState::RED)){
               //Serial.println("**********RED**************");
               Serial.print("S");
               upperSensorScored_ONS[i]= SensorState::RED;
               //delay(3000);
               fill_Block(NUM_UPPER_BLOCK_START + (i * NUM_UPPER_BLOCK_LEN), NUM_UPPER_BLOCK_LEN, CRGB::Red);    
            }
         }else if(upperDebounceBLUE[i].calculate((upperSensorState[i] == SensorState::BLUE))){
            //Serial.println("Is blue");
            if((upperSensorScored_ONS[i]!= SensorState::BLUE)){
               //Serial.println("**********BLUE**************");
               Serial.print("Y");
               upperSensorScored_ONS[i]= SensorState::BLUE;
               //delay(3000);
               fill_Block(NUM_UPPER_BLOCK_START + (i * NUM_UPPER_BLOCK_LEN), NUM_UPPER_BLOCK_LEN, CRGB::Blue);     
            }
            
         }else{
            upperSensorScored_ONS[i]= SensorState::NONE;
            fill_Block(NUM_UPPER_BLOCK_START + (i * NUM_UPPER_BLOCK_LEN), NUM_UPPER_BLOCK_LEN, MatchState_LEDs);     
         };   
      }
   }

   if(EN_CALIBRATE_PLOT){
      if((int_Calibrate>=4) & (int_Calibrate<=7)){
         lowerSensorState[int_Calibrate - 4] = getSensorState(lowerSensors[int_Calibrate - 4]);
      }   
   }else{
      //Loop through lower sensors
      for(int i=0; i <NUM_LOWER_SENSORS ; i++){
         lowerSensorState[i] = getSensorState(lowerSensors[i]);

         if(lowerDebounceRED[i].calculate((lowerSensorState[i] == SensorState::RED))){
            //Serial.println("Is red");
            if((lowerSensorScored_ONS[i]!= SensorState::RED)){
            // Serial.println("**********RED**************");
               Serial.print("X");
               lowerSensorScored_ONS[i]= SensorState::RED;
               //delay(3000);
               fill_Block(NUM_LOWER_BLOCK_START + (i * NUM_LOWER_BLOCK_LEN), NUM_LOWER_BLOCK_LEN, CRGB::Red);
            }
         }else if(lowerDebounceBLUE[i].calculate((lowerSensorState[i] == SensorState::BLUE))){
            //Serial.println("Is blue");
            if((lowerSensorScored_ONS[i]!= SensorState::BLUE)){
               //Serial.println("**********BLUE**************");
               Serial.print("H");
               lowerSensorScored_ONS[i]= SensorState::BLUE;
               //delay(3000);
               fill_Block(NUM_LOWER_BLOCK_START + (i * NUM_LOWER_BLOCK_LEN), NUM_LOWER_BLOCK_LEN, CRGB::Blue); 
            }
            
         }else{
            lowerSensorScored_ONS[i]= SensorState::NONE;
            fill_Block(NUM_LOWER_BLOCK_START + (i * NUM_LOWER_BLOCK_LEN), NUM_LOWER_BLOCK_LEN, CRGB::Black);
         };      
      }
   }
  
   //Sing Of Life Arduino
   CRGB SOL = CRGB::White;
   long int now = millis();
   //Hartbeat LED Color
   if((now - currentTime)>=50){
      SOL = CRGB::Red;
   }else if ((now - currentTime)>=25){
      SOL = CRGB::Yellow;
   }else{
      SOL = CRGB::White;
   }

   if(currentTime > hartBeatTck){
      hartBeatTck = currentTime + 500;
      hartBeat = !hartBeat;
   }

   if(hartBeat){
      leds[SIGN_OF_LIFE_AR] = SOL;
   }else{
      leds[SIGN_OF_LIFE_AR] = CRGB::Black;
   }

   //Sing Of Life Raspberry Pi
   if(signOfLifePi){
      signOfLifePi_State = !signOfLifePi_State;
      signOfLifePi = false;
   }

   if(signOfLifePi_State){
      leds[SIGN_OF_LIFE_PI] = CRGB::White;
   }else{
      leds[SIGN_OF_LIFE_PI] = CRGB::Black;
   }
   
   //Tag Lower Sensor Start Location
   leds[NUM_LOWER_BLOCK_START - 1] = CRGB::Yellow;

   // Show Match state on LEDs 3-10
   leds[matchState_int-18] = CRGB::White;
   FastLED.show();
   
   //As of 8/4/2022 this program ran at 20ms This assures this is the min
   while((millis() - currentTime)<20){}
   
   //ToDo add warning if over ?ms
}

