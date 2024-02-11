

void UpdatePressure() {
  if (updatePressure == true) {
    tympData.pressureVals[numReadingsCount] = currentPressureVal;         //set the pressure array in the struct to the current value
    updatePressure = false;                                               //prevent this function getting called again
    newMicReading = true;                                                 //allow mic to take a new reading. having this here ensures the numReadingsCount won't be updated until this one is
  }
}

void UpdateMic() {
  if (updateMic == true) {
    micAvg = micAvg / numMicReadings;             //Take average of all samples
    tympData.micVals[numReadingsCount] = micAvg;  //Store into the array in the struct for the current iteration



    CheckDifference();
    updateMic = false;
    if (numReadingsCount < numReadings) {
      numReadingsCount++;                         //update struct counter. should not exceed numReadings
    }
  }

}

void CheckDifference() {                              //Automatic Gain control function that determines if the newest sound level is higher or lower than the reference point taken at the max pressure point.
  if (micAvg - micReference > 2) {                    //compare the current microphone value (averaged). if the value is larger than some amount of error /(aka if speaker is perceived as louder than reference)
    UpdateDAC(0);                                     //then call the function to update the OTA gain / speaker volume
  }
  else if (micAvg - micReference < -2) {              //compare if the current mic value to same reference: if speaker is preceived quieter
    UpdateDAC(1);                                     //call the function to increase the volume
  }

  dac1.fastWriteA(dacOut);                          //update speaker volume
  tympData.dacVals[numReadingsCount] = dacOut;

}


void UpdateDAC(bool dacDirection) {                   //updates the speaker amplifier gain depending on the checkdifference function
  if (dacOut > 50 && dacOut < 4050) {                 //ensures the 12 bit value (0-4095) is not exceeded in either direction

    if (dacDirection == 0) {
      dacOut = dacOut - dacChange;                    //decrease speaker volume variable by the chosen interval
    }

    if (dacDirection == 1) {
      dacOut = dacOut + dacChange;                    //increase speaker volume variable by the chosen interval
    }
  }

}
