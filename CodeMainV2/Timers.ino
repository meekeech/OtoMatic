void SolenoidTimer(int wait) {
  //delay amount of time until wait value elapses
  //calculated based on volume of year (could be dynamic based on GUI input or something based on age or tymp reading or something)

  if (solenoidStart) {
    startTimeSolenoid = millis();
    solenoidStart = false;
  }
  if (millis() - startTimeSolenoid > wait) {
    solenoidStatus = 0;
    digitalWrite(MOSPIN, solenoidStatus);
  }
}

void PumpTimer(int wait) {
  //delay amount of time until wait value elapses
  //should be longer than solenoidTimer since the gasses must travel to the sensors
  if (pumpStart) {
    startTimePump = millis();
    pumpStart = false;
  }

  //add in pressure check
  if (millis() - startTimePump > wait) {
    pump1.brake();
    buttonLeftPressed = false;
    stopLoop = true;
    buttonL_SentVal = 1;
    pumpStatus = 0;
  }
}

void CheckDebounce() {

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (buttonR_Val != buttonState) {
      buttonState = buttonR_Val;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == LOW) 
        buttonR_SentVal = 0;
      else
        buttonR_SentVal = 1;
        //startDebounce = false;
        //ResetAnalogs(); //good for debugging but might want to be able to take a photo and send while you are sampling & PI can figure out the received values

      
    }
  }
  prevButtonR_Val = buttonR_Val;
}
