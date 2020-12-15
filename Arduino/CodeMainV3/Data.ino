
void UpdateWave() {
  analogWrite(A14, FullSine_8Bit[waveCount]);
  waveCount++;
  if (waveCount == 256)
    waveCount = 0;
}


void ReadAnalogs() {
  //TODO: set analog reference to 1.1V
  //pressure_Val = ads.readADC_SingleEnded(0); //should start at 2.5V or 833
  //Likely need to set a delay between microphone readings to decrease the amount of data collected
  //micArray[micCount] = analogRead(MICPIN);
  //micCount++;


  micArray[micCount] = analogRead(MICPIN);
  timeStamp[micCount] = micros();
  //Serial.println(vals[count]);
  micCount++;

  if (micCount == total) {
    micCount = 0;
    buttonLeftPressed = 0;
    
    for (int i = 0; i < total; i++) {
      Serial.print(timeStamp[i]);
      Serial.print(" ");
      Serial.println(micArray[i]);
      //delay(10);
    }

    Serial.print("Pressure: ");
    Serial.println(pressure_Val);
 

  }
}




void PressureCheck() {

  if (pressure_Val > 1023 && motorDir == 0) {
    motorDir = 1;

    //Turn off Solenoid to account for sending delay
    //Could be bad if motor switches directions before solenoid starts but should be fine
    //since 3-way valve is used instead of two way (no pressure buildup)
    digitalWrite(MOSPIN, 0);
    startTime = millis();

  }


  else if (motorDir == 1 && waitDelay == 0) {
    if (CheckTime(150, startTime)) {
      waitDelay = 1;
      digitalWrite(MOSPIN, 1);
    }
  }

  else if (pressure_Val < 453 && motorDir == 1) {
    myTimer.end();
    digitalWrite(MOSPIN, 0);
    motorDir = 0;
    buttonLeftPressed = 0;
    micDataReady = 1;

  }




}

void PressureSend() {
  //Serial.println('a');
  Serial1.print("B");
  Serial1.print(buttonL_SentVal);
  Serial1.print(buttonR_SentVal);
  Serial1.print("S");  //solenoid & pump
  Serial1.print(solenoidStatus);
  Serial1.print(motorDir);
  Serial1.print("P");
  Serial1.write(highByte(pressure_Val));
  Serial1.write(lowByte(pressure_Val));
}


void TempSend() {
  Serial1.print("B");
  Serial1.print(buttonL_SentVal);
  Serial1.print(buttonR_SentVal);
  Serial1.print("T");
  Serial1.print(objectTemp); //convert to string or something
  Serial1.print(internalTemp); //convert to string or something


}

void MicDataSend() {
  Serial1.print("M");
  Serial1.write(highByte(micArray[sendMicCount]));
  Serial1.write(lowByte(micArray[sendMicCount]));
  sendMicCount++;

  if (sendMicCount == micCount) {
    micDataReady = 0;
    micCount = 0;
  }
}



void SetAnalogs() {
  pressure_Val = 444;
}
