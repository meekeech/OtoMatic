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

#define CO2PIN A0
#define O2PIN A1
#define HUMIDPIN  A2
#define TEMPPIN A3


int temp1 = 0;
float tempVolts = 0;
float tempReal = 0;
int co2Raw;
int o2Raw;
int humidRaw;
int pumpSpeed = 80;
int photoState = 1;
int prevPhotoState = 1;
int lastDir;

Motor pump1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor fan1 = Motor(BIN1, BIN2, PWMB, 1, STBY);

char inByte = 0;
boolean newData = false;
boolean doOnce = true;
boolean buttonL_Val = 1;
boolean forwardPumping = false;
boolean reversePumping = false;
boolean startHoming = false;


//timer variables
unsigned long startTime = 0;
unsigned long endTime = 0;
int timeDif = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);
  pinMode(10, INPUT_PULLUP);

  //fan1.drive(150);
  //fan1.brake();
  //pump1.drive(-255);

  //delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:
  //  temp1 = analogRead(A3);
  //  tempVolts = temp1 * 4.8876;
  //  tempReal = (tempVolts - 500) / 10;

  //humidRaw = analogRead(HUMIDPIN);
  //o2Raw = analogRead(O2PIN);
  //co2Raw = analogRead(O2PIN);

  //Serial.print(humidRaw);
  //Serial.print(" ");
  //Serial.print("\n");

  //delay(100);

  //Serial.println(digitalRead(10));
  recvOneChar();
  showNewData();

  if (startHoming == true) {
    Homing();
  }
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
      lastDir = -1; //keep track of direction so we know what way to home
      pump1.drive(-pumpSpeed);
      forwardPumping = true;
      //fan1.brake();
    }

    if (inByte == 'r') {
      lastDir = 1;
      pump1.drive(pumpSpeed);
      reversePumping = true;
      //fan1.brake();
    }

    if (inByte == 's') {
      pump1.brake();

      //prevPhotoState = digitalRead(10);
      //startHoming = true;
      //fan1.drive(150);
      //fan1.brake();
    }

    if (inByte == 'h') {
      prevPhotoState = digitalRead(10);
      startHoming = true;
      pump1.drive((-1)*lastDir * pumpSpeed);
    }

    newData = false;
    //doOnce = true;

  }

}


void Homing() {
  photoState = digitalRead(10);
  if (photoState != prevPhotoState) {
    pump1.brake();
    startHoming = false;
  }

}
