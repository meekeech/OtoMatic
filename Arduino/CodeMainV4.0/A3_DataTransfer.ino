
void ConnectCM4() {
  Serial1.print("C");
}

void AcquireImage() {
  Serial1.print("P");
}

void MotorDirectionSend() { //change to include input parametewr for direction (makes code more readable when calling the function)
  //Serial.println("S");
  Serial1.print("D");
  Serial1.println(motorDir);
}


void StopMotor() {
  Serial1.print("X");
}

void TempSend() {
  Serial1.print("T");
  Serial1.print(",");
  Serial1.write(highByte(sentTemp));
  Serial1.write(lowByte(sentTemp));
  Serial1.print("C");
}
