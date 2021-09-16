#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_ADS1015.h>
#include "Adafruit_TPA2016.h"
#include "SparkFun_MLX90632_Arduino_Library.h"

#define NOP __asm__ __volatile__ ("nop\n\t")

#define MOSPIN 5
#define REG_ENABLE 6
#define B1PIN 8 //actual pin
#define B2PIN 7 //actual pin
#define SD_SELECT 10
//#define B1PIN 12 //test pin
//#define B2PIN 11 //test pin

#define MICPIN A0


//**************************************************************************
boolean buttonL_Val = 1;
boolean buttonL_SentVal = 1;
boolean buttonR_Val = 1;
boolean buttonR_SentVal = 1;
boolean prevButtonR_Val = 1;
boolean buttonState;
boolean startDebounce = false;
volatile boolean newMicData = false;

boolean sendTrue = false;

boolean buttonLeftPressed = false;
boolean buttonRightPressed = false;
boolean firstTimeLeft = false;

const int total = 150;
const int totalMic = 10050;
const int totalPressure = 26;
int current = 0;

int pressureArray[totalPressure];

boolean solenoidStart = false;
boolean solenoidStatus = false;
boolean motorDir = 0;
boolean waitDelay = false;
boolean sendOnce = true;

boolean flag1 = false;

volatile boolean maxPressure = false;
volatile boolean newReading = false;
volatile boolean startReading = false;
volatile boolean updateSD = false;
volatile uint16_t pressure_Val = 0;
volatile uint32_t pressure_Time = 0;

uint16_t sentTemp = 0;

//mic variables
//TODO: figure out how many dataPoints are necessary for the array

int tympCount = 0;
int tympCount2 = 0;
volatile int pressureCount = 0;
int sendtympCount = 0;
int count2 = 0;
volatile boolean micDataReady = false;
//unsigned int micArray[totalPressure][total];
unsigned int micArray[totalMic];
boolean doOnce = true;
//unsigned long micTime[total];
//unsigned long pressureTime;
unsigned long micStartTime = 0;
unsigned long micEndTime = 0;
volatile uint16_t prevPressureVal = 1020;

//timer variables
unsigned long startTime = 0;
unsigned long endTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

MLX90632 myTempSensor;
MLX90632::status errorFlag; //Declare a variable called errorFlag that is of type 'status'
float objectTemp;
float internalTemp;
byte sensorAddress = 0x3A; //modified from 0x3B becasue ADDR tied to ground

File tympFile;

IntervalTimer speakerTimer;
IntervalTimer micTimer;
IntervalTimer pressureTimer;
IntervalTimer sdTimer;


Adafruit_TPA2016 audioamp = Adafruit_TPA2016();
int waveCount = 0;
uint16_t sdCount = 0;
volatile int pressureCounter = 0;

const uint16_t FullSine_8Bit[256] =
{
  2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398,
  2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
  2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143,
  3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
  3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722,  
  3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
  3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045,
  4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
  4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065,
  4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
  3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
  3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
  3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226,
  3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
  2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496,
  2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
  2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697,
  1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311,
  1264, 1218, 1172, 1127, 1082, 1038,  995,  952,
  910,  869,  828,  788,  749,  710,  672,  636,
  600,  565,  530,  497,  465,  433,  403,  373,
  345,  318,  291,  266,  242,  219,  197,  176,
  156,  137,  120,  103,   88,   74,   61,   50,
  39,   30,   22,   15,   10,    6,    2,    1,
  0,    1,    2,    6,   10,   15,   22,   30,
  39,   50,   61,   74,   88,  103,  120,  137,
  156,  176,  197,  219,  242,  266,  291,  318,
  345,  373,  403,  433,  465,  497,  530,  565,
  600,  636,  672,  710,  749,  788,  828,  869,
  910,  952,  995, 1038, 1082, 1127, 1172, 1218,
  1264, 1311, 1358, 1405, 1453, 1501, 1550, 1599,
  1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997
};
//**************************************************************************

Adafruit_ADS1015 ads;

int16_t adc0;

struct myData {
  //volatile uint32_t pressureTime;
  volatile uint16_t pressureVal;
  //volatile uint32_t micTime;
  volatile uint16_t micVal[total];

};

struct myData tympData;

void setup() {

  speakerTimer.priority(100);
  micTimer.priority(100);
  pressureTimer.priority(200);
  //sdTimer.priority(152);



  Serial.begin(115200);
  Serial1.begin(115200);
  delay(200);

  Wire.begin();

  pinMode(B1PIN, INPUT_PULLUP);
  pinMode(B2PIN, INPUT_PULLUP);
  pinMode(MOSPIN, OUTPUT);
  pinMode(REG_ENABLE, OUTPUT);

  //SD Card initialization
  if (!SD.begin(SD_SELECT)) {
    Serial.println("initialization failed!");
    return;
  }

  // Analog to Digital Converter
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
  ads.begin();

  analogWriteResolution(12);
  analogReadResolution(10);
  //16 is very noisy, try 14, 12 (10 is good)

  AmpSetup();
  //TempSetup();

  //SetAnalogs();

  digitalWrite(REG_ENABLE, LOW);
  digitalWrite(MOSPIN, LOW);




  //delay(6000);
  //micStartTime = millis();

  //1000Hz -> 3.9
  //678 -> 5.76
  //666Hz -> 5.92
  //226 -> 17.2
  tympFile = SD.open("tymp.dat", O_CREAT | O_WRITE | O_TRUNC);
  delay(500);
  Serial.println("Ready");
}

void loop() {
  buttonL_Val = digitalRead(B1PIN);
  buttonR_Val = digitalRead(B2PIN);

  if (buttonL_Val == 0) {
    digitalWrite(MOSPIN, HIGH);
    buttonLeftPressed = 1;
    speakerTimer.begin(UpdateWave, 17.2);
    delay(2000);
    MotorDirectionSend();
    pressureTimer.begin(ReadPressure, 400);
  }

  if (buttonLeftPressed == true) {
    //s
    //Serial.println(tympData.pressureVal);
    //Open Valve toward ear and set status to on
    if (maxPressure == true) {
      Serial.println("max");
      startReading = true;
      newReading = false;
      maxPressure = false;
      micTimer.begin(ReadMic, 30);
    }

    if (updateSD == true) {
      sdUpdate();
      sdCount++;
      updateSD = false;
    }

    if (motorDir == 1 && sendOnce == true) {
      MotorDirectionSend();
      sendOnce = false;
    }

    if (micDataReady == true) {

      StopMotor();
      speakerTimer.end();
      micTimer.end();
      pressureTimer.end();
      tympFile.close();
      tympFile = SD.open("tymp.dat", FILE_READ);
      buttonLeftPressed = false;
      micDataReady = false;
      sendOnce = true;
      startReading = false;

      MicDataPrint();
      sdCount = 0;
      prevPressureVal = 1020;

    }
  }


  if (buttonR_Val == 0) {
    //digitalWrite(REG_ENABLE, LOW);
    //delay(2000);
    //GetTemp();
    //TempSend();
    micDataReady = true;

    //digitalWrite(REG_ENABLE, HIGH);


  }


}
