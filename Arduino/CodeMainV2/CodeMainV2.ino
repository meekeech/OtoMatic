#include <SparkFun_TB6612.h>

//Handle definitions
#define CO2PIN A0
#define O2PIN A1
#define PPIN A2
#define HPIN A3
#define B1PIN A4 //left
#define B2PIN A5 //right
#define FPIN A6
#define MOSPIN 3

//Motor definitions
#define STBY 8
#define AIN1 9
#define AIN2 10
#define PWMA 11

Motor pump1 = Motor(AIN1, AIN2, PWMA, 1, STBY);

boolean buttonL_Val = 1;
boolean buttonL_SentVal = 1;
boolean buttonR_Val = 1;
boolean buttonR_SentVal = 1;
boolean prevButtonR_Val = 1;
boolean buttonState;
boolean startDebounce = false;

int carbon_Val = 0;
int oxy_Val = 0;
int pressure_Val = 0;
int humid_Val = 0;
int flow_Val = 0;

int solenoidTime = 1000; //1 seconds to sample ear volume
int pumpTime = 3000; //10 seconds to allow flow

boolean sendTrue = false;
boolean analogTrue = false;
boolean solenoidStart = false;
boolean pumpStart = false;
boolean solenoidStatus = false;
boolean pumpStatus = false;

boolean buttonLeftPressed = false;
boolean buttonRightPressed = false;
boolean firstTimeLeft = false;

boolean stopLoop = false;
boolean startSend = false;
boolean startRead = false;
boolean lastSend = false;
boolean takePhoto = false;

//timer variables
unsigned long startTimeSolenoid = 0;
unsigned long endTimeSolenoid = 0;

unsigned long startTimePump = 0;
unsigned long endTimePump = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(9600);
  pinMode(B1PIN, INPUT_PULLUP);
  pinMode(B2PIN, INPUT_PULLUP);
  pinMode(MOSPIN, OUTPUT);

}

void loop() {
  ReadAnalogs();
  SerialSend();  //put this in a timer

  buttonL_Val = digitalRead(B1PIN);
  buttonR_Val = digitalRead(B2PIN);


  //Starts sampling gasses: sent value will be B10CxxOxxHxxPxxFxx
  if (buttonL_Val == LOW) {
    buttonLeftPressed = true;
    solenoidStart = true;
    pumpStart = true;
    solenoidStatus = 1;
    pumpStatus = 1;
    digitalWrite(MOSPIN, solenoidStatus);
    pump1.drive(255);
  }

  if (buttonLeftPressed) {
    //CheckDebounce(B2PIN);

    SolenoidTimer(solenoidTime);
    PumpTimer(pumpTime);

  }


  if (buttonR_Val != prevButtonR_Val) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  CheckDebounce();


  //startRead = false;

}
