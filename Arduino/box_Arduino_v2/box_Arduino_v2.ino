#include <SparkFun_TB6612.h>

#define NOP __asm__ __volatile__ ("nop\n\t")

//Motor definitions
#define STBY 9
#define AIN1 8
#define AIN2 7
#define PWMA 6
#define BIN1 3
#define BIN2 4
#define PWMB 5

Motor pump1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor fan1 = Motor(BIN1, BIN2, PWMB, 1, STBY);

char inByte = 0;
boolean newData = false;
boolean doOnce = true;
boolean buttonL_Val = 1;


//timer variables
unsigned long startTime = 0;
unsigned long endTime = 0;
int timeDif = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:


  recvOneChar();
  showNewData();
}


void recvOneChar() {
  if (Serial.available() > 0) {
    inByte = Serial.read();
    NOP;
    newData = true;
  }
}

void showNewData() {
  if (newData == true) {
    if (inByte == 'f') {
      //startMotor = true;
      pump1.drive(255);


    }
    newData = false;
    doOnce = true;
  }
}
