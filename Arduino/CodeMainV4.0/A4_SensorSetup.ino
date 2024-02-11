

void ClockSetup() {
  if (clockgen.begin() != ERROR_NONE)
  {
    /* There was a problem detecting the IC ... check your connections */
    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  clockgen.setupPLLInt(SI5351_PLL_A, 28);
  clockgen.setupMultisynth(0, SI5351_PLL_A, 28, 0, 1);

  clockgen.setupPLL(SI5351_PLL_B, 27, 14757, 15625);
  clockgen.setupMultisynth(1, SI5351_PLL_B, 66, 900, 1);
  clockgen.setupRdiv(1, SI5351_R_DIV_32);

}


void TempSetup() {
  myTempSensor.begin(TEMPADR, Wire, errorFlag);

  if (errorFlag == MLX90632::SENSOR_SUCCESS)  Serial.println("MLX90632 online!");
  if (errorFlag == MLX90632::SENSOR_ID_ERROR) Serial.println("Sensor ID did not match the sensor address. Probably a wiring error.");
  else if (errorFlag == MLX90632::SENSOR_I2C_ERROR) Serial.println("Sensor did not respond to I2C properly. Check wiring.");
  else if (errorFlag == MLX90632::SENSOR_TIMEOUT_ERROR) Serial.println("Sensor failed to respond.");
  else Serial.println("Other Error");

  //myTempSensor.continuousMode();

}


void EnableSpeaker() {
  Serial.println("Starting Speaker");
  clockgen.enableOutputs(true);   // Turn ON the output - it defaults to OFF
  delay(200);
  wavegen.EnableOutput(true);
}


void EnableValve(const uint8_t solPin, bool solState) {       //enables or disables valves depending on input
  Serial.println("Changing Valve");
  //digitalWrite(SOLPIN1, HIGH);
  digitalWrite(solPin, solState);
}
