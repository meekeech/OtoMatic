//
//void ReadPressure() {
//  //Should be re-written to reduce redundancy and follow a more state-like approach
//
//  if (newMicReading == false) {
//
//    currentPressureVal = analogRead(PRESSUREPIN);
//
//    if (currentPressureVal > maxPressureVal && motorDir == 0) {                                               //compare the current pressure reading with the max value set - also ensures motor direction hasn't changed
//      maxPressure = true;                                                                                     //let main loop know max pressure was reached so it can start the mic thread and take a reference mic reading
//      motorDir = 1;                                                                                           //change motor direcion
//    }
//
//    if (startReading == true && (prevPressureVal - currentPressureVal) >= pressureInterval) {                 //taking readings every time pressure changes by 20 - gives about 13 values (250 pressure values/20)
//      updatePressure = true;                                                                                  //tell main loop to record the current pressure into the main struct that tracks mic and pressure vals
//      prevPressureVal = currentPressureVal;                                                                   //assign the current value as the new previous value for next comparison
//    }
//
//    if ((currentPressureVal < minPressureVal && motorDir == 1)  || numReadingsCount > numReadings) {          //determine if pressure has dropped below the minimum defined value, or if the number of readings exceeds what is defined in the struct arrays
//      startReading = false;
//      motorDir = 0;                                                                                           //reverse motor direction again. Called here to ensure this segment isn't repeated (main might not update before another iteration)
//      returnHome = true;
//    }
//
//    if (returnHome == true && currentPressureVal > startingPressureVal) {                                     //compare current pressure to the intial reading
//      returnHome = false;
//      EnableValve(SOLPIN2, 0);
//      micDataReady = true;                                                                                    //Tell main loop that the mic data is ready to be sent or displayed//disable the solenoid to prevent any further airflow
//    }
//  }
//}

void ReadPressure() {
  if (newMicReading == false) {
    currentPressureVal = analogRead(PRESSUREPIN);               
    switch (TympState) {
      
      case States::FindMax:
        if (sendOnce == true) {
          //Serial.println("FindMax");
          sendOnce = false;
        }

        if (currentPressureVal > maxPressureVal) {                                               //compare the current pressure reading with the max value set - also ensures motor direction hasn't changed
          maxPressure = true;                                                                     //let main loop know max pressure was reached so it can start the mic thread and take a reference mic reading
          motorDir = 1;                                                                           //change motor direcion
        }
        break;

      case States::FindMin:
        if (sendOnce == true) {
          //Serial.println("FindMin");
          sendOnce = false;
        }
        if (currentPressureVal < minPressureVal || numReadingsCount >= numReadings ) {                                             //determine if pressure has dropped below the minimum defined value, or if the number of readings exceeds what is defined in the struct arrays
          startReading = false;
          motorDir = 0;                                                                        //reverse motor direction again. Called here to ensure this segment isn't repeated (main might not update before another iteration)
          returnHome = true;
        }
        else {
          if ((prevPressureVal - currentPressureVal) >= pressureInterval) {                 //taking readings every time pressure changes by 20 - gives about 13 values (250 pressure values/20)
            updatePressure = true;                                                                                  //tell main loop to record the current pressure into the main struct that tracks mic and pressure vals
            prevPressureVal = currentPressureVal;                                                                   //assign the current value as the new previous value for next comparison
          }
        }
        break;

      case States::Homing:
        if (sendOnce == true) {
          //Serial.println("Homing");
          sendOnce = false;
        }
        if (currentPressureVal > startingPressureVal) {                                     //compare current pressure to the intial reading
          EnableValve(SOLPIN2, 0);
          micDataReady = true;                                                               //Tell main loop that the mic data is ready to be sent or displayed//disable the solenoid to prevent any further airflow
          TympState = States::Stopping;
        }
        break;

        //case States::Stopping:

    }
  }
}

void ReadMic() {
  if (newMicReading == true) {
    if (micCount < numMicReadings) {                //take several samples of mic reading to average out the noise
      micNew = analogRead(MICPIN);                  //get latest readings
      micAvg = micAvg + micNew;                     //calculate the Cummulative sum
      micCount++;                                   //update the sample counter
    }
    else {
      newMicReading = false;
      updateMic = true;
      micCount = 0;                                 //Reset sample counter
    }
  }
}

void ReadTemp() {
  objectTemp = myTempSensor.getObjectTemp();
  //internalTemp = myTempSensor.getSensorTemp();
  sentTemp = objectTemp * 100;
  Serial.println(objectTemp);
}
