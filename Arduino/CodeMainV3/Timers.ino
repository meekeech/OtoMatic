


boolean CheckTime(int duration,unsigned long initial) {
  if (millis()-initial > duration) {
    return 1;
  }
  else
    return 0;
  
  
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
