

void loop() {
  unsigned long datos = ps2x.read();    // Never comment this line
  timeManagement();
  serialEvent();

  if(runTime > 150){
    //buttonPress();
    homing();                 // Cross
    standUpDown();            // Triangle
    toggleStateMachine();     // R2 and L2
    //reset_esp_now();          // Square
  }


  
  if(stateMachine == 0){
    kinematicsPeriod += timeDif;
    if(kinematicsPeriod >= kinematicsFreq){
      kinematicsPeriod = 0;
      //moveInverseKinematics(2.51, 2.25, 2.01);
      holdInverseKinematics(2.5, 70, 50);
    }    
  }

  else if(stateMachine == 1){
    walkingPeriod += timeDif;
    if(walkingPeriod >= walkingFreq){
      walkingPeriod = 0;
      walking(); //Step section time [miliseconds], step in X, step in Y, degrees in Yaw
    }
  }


  /*
  jumpPeriod += timeDif;
  if(jumpPeriod >= jumpInterval){
    jumpPeriod = 0;
    jump();
  }
  */

  //if (torqueTest) torqueTesting();
}

void timeManagement(){
  //Time managment for DEMOs' movements
  runTime = millis();
  timeDif = runTime - prevT;
  prevT = runTime;
}

void serialEvent() {
  // a string to hold incoming data
  static String inputString;
  while(Serial.available()){
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    inputString += inChar;
    // end of user input
    if (inChar == '\n') {
      if(inputString.charAt(0) == 'T'){
       //sendData("M", inputString.substring(1).toFloat(), HT1);
       //sendData("M", inputString.substring(1).toFloat(), KT1);
       //Serial.println(inputString);
      }
      else if(inputString.charAt(0) == 'I'){
        jumpInterval = inputString.substring(1).toFloat();
      }
      else if(inputString.charAt(0) == 'J'){
        torqueTest = true;
      }
      else{
        if (jl){
          sendData(0, inputString, 0, inputString, 0, inputString, 0); 
        }
       

        if (frl){
          sendData(1, inputString, 0, inputString, 0, inputString, 0);
        }
  
        if (fll){
          sendData(2, inputString, 0, inputString, 0, inputString, 0);
        }

        if (bll){
          sendData(3, inputString, 0, inputString, 0, inputString, 0);
        }
      
      
       Serial.println(inputString);
      } // Send any command to the actuators
      inputString = "";
    }
  }
}
