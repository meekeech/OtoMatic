void MotorDirectionSend() {
  //Serial.println("S");
  Serial1.print("D");
  Serial1.print(motorDir);
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
