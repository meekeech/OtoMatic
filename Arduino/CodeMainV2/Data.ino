
void ReadAnalogs() {
  carbon_Val = analogRead(CO2PIN);
  oxy_Val = analogRead(O2PIN);
  pressure_Val = analogRead(PPIN);
  humid_Val = analogRead(HPIN);
  flow_Val = analogRead(FPIN);
}

void ResetAnalogs() {
  carbon_Val = 0;
  oxy_Val = 0;
  pressure_Val = 0;
  humid_Val = 0;
  flow_Val = 0;
}


void SetAnalogs() {
  carbon_Val = 111;
  oxy_Val = 222;
  pressure_Val = 444;
  humid_Val = 333;
  flow_Val = 555;
}

void SerialSend() {

  if (Serial.available() > 0) {
    if (Serial.read() == 'r') {
      Serial.print("B");
      Serial.print(!buttonL_Val);
      Serial.print(!buttonR_SentVal);
      Serial.print("S");  //solenoid & pump
      Serial.print(solenoidStatus);
      Serial.print(pumpStatus);
      Serial.print("C");
      Serial.write(highByte(carbon_Val));
      Serial.write(lowByte(carbon_Val));
      Serial.print("O");
      Serial.write(highByte(oxy_Val));
      Serial.write(lowByte(oxy_Val));
      Serial.print("H");
      Serial.write(highByte(humid_Val));
      Serial.write(lowByte(humid_Val));
      Serial.print("P");
      Serial.write(highByte(pressure_Val));
      Serial.write(lowByte(pressure_Val));
      Serial.print("F");
      Serial.write(highByte(flow_Val));
      Serial.write(lowByte(flow_Val));
      Serial.println();
    }
  }
}
