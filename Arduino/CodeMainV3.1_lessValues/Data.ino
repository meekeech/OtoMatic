void GetTemp() {
  objectTemp = myTempSensor.getObjectTemp();
  //internalTemp = myTempSensor.getSensorTemp();
  sentTemp = objectTemp * 100;
  Serial.println(objectTemp);
}

void UpdateWave() {
  analogWrite(A14, FullSine_8Bit[waveCount]);
  waveCount++;
  if (waveCount == 256)
    waveCount = 0;
}

void ReadMic() {
  if (newReading == true) {
    if (tympCount < total) {
      tympData.micVal[tympCount] = analogRead(A0);
      tympCount++;

    }
    else {
      tympCount = 0;
      newReading = false;
      updateSD = true;
    }
  }


}

void ReadPressure() {

  if (newReading == false) {

    pressure_Val = ads.readADC_SingleEnded(0);

    if (pressure_Val > 1020 && motorDir == 0) {
      maxPressure = true;
      motorDir = 1;
    }

    //taking readings every time pressure changes by 20 - gives about 26 values (520 pressure values/20)
    if (startReading == true && pressure_Val > 0 && (prevPressureVal - pressure_Val) >= 20) {
      tympData.pressureVal = pressure_Val;
      newReading = true;
      prevPressureVal = pressure_Val;
      pressureCount++;
    }
    if ((pressure_Val < 500 && pressure_Val > 0 && motorDir == 1)  || pressureCount > 25) {
      //453
      digitalWrite(MOSPIN, LOW);
      motorDir = 0;
      micDataReady = true;
    }
  }
}


void sdUpdate() {
  if (tympFile) { //this takes about 0.4ms to complete
    tympFile.write((const uint8_t *)&tympData, sizeof(tympData));
  }
}


void MicDataPrint() {
  //Serial.println


  for (int i = 0; i < sdCount; i++) {

    if (tympFile.available()) {
      tympFile.read((uint8_t *)&tympData, sizeof(tympData));
    }

    Serial.print(tympData.pressureVal);
    Serial.print(" ");

    for (int j = 0; j < total; j++) {
      Serial.print(tympData.micVal[j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  tympFile.close();

  tympFile = SD.open("tymp.dat", FILE_READ);
  for (int i = 0; i < sdCount; i++) {

    if (tympFile.available()) {
      tympFile.read((uint8_t *)&tympData, sizeof(tympData));
    }

    for (int j = 0; j < total; j++) {
      Serial.println(tympData.micVal[j]);
    }
  }

  tympFile.close();

}
