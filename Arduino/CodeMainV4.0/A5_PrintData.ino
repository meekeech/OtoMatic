void PrintPressure() {
     Serial.println(analogRead(PRESSUREPIN));
     //Serial.println(analogRead(POTPIN));
}


void PrintData() {

  Serial.print("Mic Reference: ");
  Serial.println(micReference);
  Serial.println("P_raw P_real Mic Dac MicRef MicRefMax MicRefMin");
  for (int i = 0; i < numReadings; i++) {
    Serial.print(tympData.pressureVals[i]);
    Serial.print(" ");
    Serial.print(((((tympData.pressureVals[i]*(0.00495387)/5)-0.5)/0.057)*100));
    Serial.print(" ");
    Serial.print(tympData.micVals[i]);
    Serial.print(" ");
    Serial.print(tympData.dacVals[i]);
    Serial.print(" ");
    Serial.print(micReference);
    Serial.print(" ");
    Serial.print(micReference+2);
    Serial.print(" ");
    Serial.println(micReference-2);
    
  }

}


//seperate thread that prints current pressure val at a slow interval
//only will work if the variable is passed through a queue to this thread from the ReadPressure one
