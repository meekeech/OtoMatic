/*Version four of the main code that runs on the Teensy (4.0 for this iteration, which is Otoscope version 3.0)
   Interfaces with the mic, speaker, pressure sensor, camera light, temperature sensor, solenoids
   Button1: Sends a signal for taking a photo
   Button2: Initiates Tympanometry process including the automatic gain control circuit & Sends data
   Button3: Takes a Temperature Reading & sends data

   Solenoid States
    Sol1 ON, Sol2 ON: Pump to/from Ear
    Soll ON, Sol2 OFF: Pump Blocked, Ear to Atmosphere (AVOID)
    Sol1 OFF, Sol2 ON: Pump to/from Atmosphere, Ear Blocked
    Sol1 OFF, Sol 2 OFF: Pump to/from Atmosphere, Ear to Atmosphere


  Power Supply Colours
  Red, Black, Yellow Blue (12 -12 5 G)
*/


#include <Wire.h>
#include <SPI.h>
#include "SparkFun_MLX90632_Arduino_Library.h"
#include <Adafruit_SI5351.h>
#include <AD9833.h>
#include "MCP_DAC.h"

#define B1PIN 3
#define B2PIN 4
#define B3PIN 5
#define LED1PIN 6
//#define SOLPIN1 7
//#define SOLPIN2 8
#define PRESSUREPIN A9
#define WAVEPIN A8
#define DACPIN A7
#define MICPIN A6
#define POTPIN A0

#define TEMPADR 0x3A //usually 3B is default but ADDR tied to GND
#define NOP __asm__ __volatile__ ("nop\n\t")

bool b1Val = 1;
bool b2Val = 1;
bool b3Val = 1;
bool b1Pressed = 0;
bool startTympanometry = 0;
bool b3Pressed = 0;
bool motorDir = 0;
bool sendOnce = 1;
bool serial1Connected = 0;

volatile bool maxPressure = false;
volatile bool startReading = false;
volatile bool micDataReady = false;
volatile bool newMicReading = false;
volatile bool updateMic = false;
volatile bool updatePressure = false;
volatile bool returnHome = false;
enum States {FindMax, FindMin, Homing, Stopping};
volatile char TympState = States::FindMax;

//Modifiable parameters
const uint8_t numMicReadings = 10; //size of array for calculating average of micRMS values
const uint8_t pressureInterval = 1; //how often to take readings
uint16_t dacChange = 20;  //step size between speaker gain changes

const uint8_t SOLPIN1 = 7;    //defined as constants rather than using "define" so it can be passed in function
const uint8_t SOLPIN2 = 8;    //might be another way to keep it as defined

//Official Values
//maxPressureVal = initPressureVal + 115 = 627 for 512 initial
//minPressureVal = initPressureVal - 230 = 282 for 512 initial
const uint16_t maxPressureVal = 600;
const uint16_t minPressureVal = 380;

//calculate how many Readings for the main tymp Array size based on max,min, and pressure interval.
//Take ceiling value to ensure there's always more space than the fractional amount.
const uint16_t numReadings = ceil((maxPressureVal - minPressureVal) / pressureInterval); //13

struct myData {
  volatile uint16_t pressureVals[numReadings];
  volatile uint16_t micVals[numReadings];
  volatile uint16_t dacVals[numReadings];
};

volatile uint16_t currentPressureVal;
volatile uint16_t startingPressureVal;
uint16_t sentTemp = 0;
uint16_t micArray[numMicReadings];
uint16_t micReference = 0;
uint16_t micAvg;
uint16_t micNew = 0;
volatile uint16_t numReadingsCount = 0;
uint16_t dacOut = 2047;

uint8_t micCount = 0;


volatile uint16_t pressureCount = 0;
uint16_t prevPressureVal = maxPressureVal;

float objectTemp;

//Debugging Serial Variables
char inByte;
char inByte1;
bool newData = false;
bool newData1 = false;

MLX90632 myTempSensor;
MLX90632::status errorFlag;

MCP4921 dac1;
AD9833 wavegen(WAVEPIN);
Adafruit_SI5351 clockgen = Adafruit_SI5351();
struct myData tympData;

IntervalTimer micTimer;
IntervalTimer pressureTimer;
IntervalTimer printTimer;



void setup() {
  wavegen.Begin();                  //start sine wave generator (AD9833)
  delay(200);
  Wire.begin();                     //Wire enables i2c comms on pins 18&19
  Wire1.begin();                    //Wire1 enables i2c comms on pins 16 & 17
  Serial.begin(115200);             //Enables communication through USB for serial monitor access
  Serial1.begin(115200);            //Enables communication trhough pins 0 and 1 (communicate with Compute Module)
  delay(200);
  dac1.begin(A7);                   //Start digital to analog converter IC for OTA control (speaker volume)
  delay(200);

  micTimer.priority(100);           //Set priority for threads that run microphone data and pressure data acquisition
  pressureTimer.priority(150);
  printTimer.priority(255);

  analogReadResolution(10);         //Teensy had ADC resolution up to 12 bits but last two are noisy

  pinMode(B1PIN, INPUT_PULLUP);     //button pins required input pullup resistor activation (no external one used)
  pinMode(B2PIN, INPUT_PULLUP);
  pinMode(B3PIN, INPUT_PULLUP);
  pinMode(LED1PIN, OUTPUT);         //LED control requires output pin (acts as source)
  //pinMode(SOLPIN1, OUTPUT); //not working currently (wire needs direct connection)
  pinMode(SOLPIN2, OUTPUT);         //Solenoid valve requires output pin

  //Usually don't need to assign pinmode to analog pins but
  //seems required on teensy4.0 so that there's no hysterisis
  pinMode(A9, INPUT);
  pinMode(A6, INPUT);
  pinMode(A0, INPUT);

  ClockSetup();                     //setup the clock generators for the sine wave generator and the band pass filter (MF10)
  TempSetup();                    //setup the temperature sensor

  //Define frequency used in sine wave generator - 226 used most often but can adjust this on the fly
  wavegen.ApplySignal(SINE_WAVE, REG0, 227); //off by 1 Hz (inaccuracy of clk input)

  //Set the baseline gain of OTA/speaker (volume)
  dac1.fastWriteA(dacOut);



  analogWrite(LED1PIN, 255);        //set brightness of LED (0-255 bits)


  Serial.println("Setup Done");

  EnableSpeaker();

  printTimer.begin(PrintPressure, 100);

}

void loop() {

  //Put this into a thread so it doesn't affect the other stuff?
  if (serial1Connected == false) {
    recvOneChar1();
    showNewData1();


  }

  //Read button states and assign to variables
  b1Val = digitalRead(B1PIN);
  b2Val = digitalRead(B2PIN);
  b3Val = digitalRead(B3PIN);



  if (b1Val == 0) {
    delay(100);
    EnableValve(SOLPIN2, 0);
    //Send Photo Command
    //AcquireImage();
    //EnableValve(SOLPIN2, 1);
  }

  if (b2Val == 0 && serial1Connected == true) {
    delay(100);                                   //replace with debounce code
    startTympanometry = 1;
    startingPressureVal = analogRead(PRESSUREPIN);//Take an intial pressure reading: used for returning to "home" position after tympanometry (deloads the ear from large negative pressure)
    EnableValve(SOLPIN2, 1);                      //Open the channel between the pump and the ear
    delay(1000);
    EnableSpeaker();                              //Start playing a sound (not required until later but good for debugging)
    MotorDirectionSend();                         //Move pump 'forward' aka the physical flag goes upward and a positive pressure applied to ear
    pressureTimer.begin(ReadPressure, 50);       //Start the thread for reading pressure at 400uS intervals
  }

  if (b3Val == 0) {
    delay(100);
    //TempSend();                                   //Send the temperature with a 10x scale to remove decimal points. Should be converted to base 2 system eventually
    Serial.println(myTempSensor.getObjectTemp());
  }

  ButtonLoop();
  UpdateMic();
  UpdatePressure();

}

void ButtonLoop() {

  if (startTympanometry == true) {
    if (maxPressure == true) {                //Pump with positive pressure until defined max pressure is reached
      Serial.println("max");
      printTimer.end();
      startReading = true;
      TympState = States::FindMin;
      sendOnce = true;
      newMicReading = false;
      maxPressure = false;

      //Take the reference value that will be used as comparison for all future values. Eardrum is assumed to be practically rigid, and all sound reflected: no sound lost to compliance
      micReference = analogRead(MICPIN);
      micTimer.begin(ReadMic, 30);              //Start Microphone readings at 30us intervals
      MotorDirectionSend();                     //Send command to reverse pump direction (flag does down, negative pressure, or sucking applied to ear canal)
    }

    if (returnHome == true) {
      micTimer.end();                           //end mic thread first to stop taking readings. continue reading pressure vals to know when to stop the motor
      MotorDirectionSend();                     //reverse the motor again
      TympState = States::Homing;
      sendOnce = true;
      returnHome = false;

    }

    if (micDataReady == true) {                 //Activated when the lowest defined pressure is reached
      StopMotor();                              //send command to stop the pump. Replace with command that reverses pump again until 0 differential pressure is read
      // pressureTimer.end();                      //end the pressure thread
      startTympanometry = false;                    //reset variables for repeated use
      micDataReady = false;
      startReading = false;
      prevPressureVal = maxPressureVal;         //reset previous value to be the highest possible value that is defined
      PrintData();                         //print values to screen for debugging

    }
  }

}
