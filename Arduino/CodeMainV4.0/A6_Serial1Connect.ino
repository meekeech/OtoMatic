
void recvOneChar1() {
  if (Serial1.available() > 0) {
    inByte1 = Serial1.read();
    //delayMicroseconds(1);
    NOP;
    newData1 = true;
  }
}

void showNewData1() {
  if (newData1 == true) {
    if (inByte1 == 'A') {
      Serial1.println("B");
      serial1Connected = true;
      Serial.println("Compute Module 4 Connected");
    }
    if (inByte1 == 'C') {
      serial1Connected = false;
      Serial.println("Compute Module 4 Disconnected");
    }
    newData1 = false;
  }
}
