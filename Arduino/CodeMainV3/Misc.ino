
void TempSetup() {
  myTempSensor.begin(sensorAddress, Wire, errorFlag);

  if (errorFlag == MLX90632::SENSOR_SUCCESS)  Serial.println("MLX90632 online!");
  if (errorFlag == MLX90632::SENSOR_ID_ERROR) Serial.println("Sensor ID did not match the sensor address. Probably a wiring error.");
  else if (errorFlag == MLX90632::SENSOR_I2C_ERROR) Serial.println("Sensor did not respond to I2C properly. Check wiring.");
  else if (errorFlag == MLX90632::SENSOR_TIMEOUT_ERROR) Serial.println("Sensor failed to respond.");
  else Serial.println("Other Error");

  //myTempSensor.continuousMode();

}


void AmpSetup() {
  audioamp.begin();
  audioamp.setAGCCompression(TPA2016_AGC_OFF);
  audioamp.setReleaseControl(0);
  audioamp.setGain(0);
}
