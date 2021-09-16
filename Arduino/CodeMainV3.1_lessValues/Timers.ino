


boolean CheckTime(int duration,unsigned long initial) {
  if (millis()-initial > duration) {
    return 1;
  }
  else
    return 0;
  
  
}
