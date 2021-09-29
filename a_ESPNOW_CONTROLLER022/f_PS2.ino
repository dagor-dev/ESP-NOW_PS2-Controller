
void toggleStateMachine(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 1000){
    pressedPeriod = 0;
    pressed = false;
  }
  if(ps2x.button(PSB_R2) && !pressed){
    pressed = true;
    Serial.println("STATE MACHINE: Walking.");
    stateMachine = 1;
  }
  else if(ps2x.button(PSB_L2) && !pressed){
    pressed = true;
    Serial.println("STATE MACHINE: Kinematics Demo.");
    stateMachine = 0;
  }
}

void reset_esp_now(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 5000){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if(ps2x.button(PSB_SQUARE) && !pressed){
      Serial.println("[SQUARE] RESETTING ESP-NOW.");
      pressed = true;

      espNowDeinit();
      delay(2000);
      espNowInit();

      Serial.println("[SQUARE] DONE.");
  }
}

void homing(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 1000){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if(ps2x.button(PSB_CROSS) && !pressed){
      Serial.println("[X] SET POSITION AS HOME.");
      pressed = true;
      
      if (frl){
          sendData(0, "home", 0, "home", 0, "home", 0);
      }

      if (fll){
          sendData(1, "home", 0, "home", 0, "home", 0);
      }

      if (brl){
          sendData(2, "home", 0, "home", 0, "home", 0);
      }

      if (bll){
          sendData(3, "home", 0, "home", 0, "home", 0);
      }

      if (jl){
          sendData(0, "home", 0, "home", 0, "home", 0);
      }
      
      if (tt){
          sendData(0, "home", 0, "home", 0, "home", 0); 
      }
  }
}

void standUpDown(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  static bool standDirection = false;
  static float positionX = 0;
  static float positionY = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 5000){
    pressedPeriod = 0;
    pressed = false;
  }
  if(ps2x.buttonNewState(PSB_TRIANGLE) && !pressed){
    pressed = true;

    if (!standDirection){
      Serial.println("STANDING UP.");
      standDirection = true;
      for (int positionZ = 0; positionZ <=240; positionZ++){
      //Calculate IK
        inverseKinematics(&legs[0], positionZ,  positionX,  positionY);
        inverseKinematics(&legs[1], positionZ,  positionX, -positionY);
        inverseKinematics(&legs[2], positionZ, -positionX,  positionY);
        inverseKinematics(&legs[3], positionZ, -positionX, -positionY);

        sendCalculatedAngles();
        delay(10);
      }      
    }
    else if (standDirection){
      Serial.println("STANDING DOWN.");
      standDirection = false;
      for (int positionZ = 240; positionZ >=0; positionZ--){
        //Calculate IK
        inverseKinematics(&legs[0], positionZ,  positionX,  positionY);
        inverseKinematics(&legs[1], positionZ,  positionX, -positionY);
        inverseKinematics(&legs[2], positionZ, -positionX,  positionY);
        inverseKinematics(&legs[3], positionZ, -positionX, -positionY);

        sendCalculatedAngles();
        delay(10);
      }
    } //standDirection
  } // !pressed
}

/*
void buttonPress(){
  unsigned long datos = ps2x.read();

  // R1 BUTTON PRESS -> Read right analog stick and command 1 actuator
  if(ps2x.button(PSB_R1)){
    //a0 = ps2x.analog(0);
    //a1 = ps2x.analog(1);
    //multiplier2 += ((a0-128)/128) * constant;
    //if (multiplier2 <= 0) multiplier2 = 0;
    //adder2 = -((a1-128)/128);
    //target2 += adder2*multiplier2;
    
    /*Serial.print(adder2);
    Serial.print(", ");
    Serial.print(multiplier2);
    Serial.print(", ");
    Serial.println(target2);
    
    //Send target with ESPNOW
    //sendData("M",target2,KT1);
    delay(5); 
  }

  // PAD RIGHT BUTTON PRESS -> SET VOLTAGE LIMIT TO 2.5
  else if(ps2x.buttonNewState(PSB_PAD_LEFT) && left == false){
      Serial.println("Change voltage limit: 2.2.");
      //sendData("demo", 0, HT1);
      //sendData("demo", 0, KT1);
  }
  else if(ps2x.buttonNewState(PSB_PAD_LEFT) && left == true){
      left = false;
  }


  // TRIANGLE BUTTON PRESS -> START JUMP DEMO
  if(ps2x.buttonNewState(PSB_TRIANGLE) && triangle == false){
      Serial.println("Jumping start");
      jumping = true;
  }
  else if(ps2x.buttonNewState(PSB_TRIANGLE) && triangle == true){
      triangle = false;
  }


  // PAD UP BUTTON PRESS -> SET VOLTAGE LIMIT TO 3
  else if(ps2x.buttonNewState(PSB_PAD_UP) && up == false){
      Serial.println("Change voltage limit: 2.5.");
      if (jl){
        //sendData("MQL", 3.0, HT1);
        //sendData("MDL", 3.0, HT1);
        //sendData("MQL", 3.0, KT1);
        //sendData("MDL", 3.0, KT1);
      }
      //sendData2("MAP", 20.0);
      //sendData2("MQR", 50.0);
      //sendData2("MQR", 50.0);
  }
  else if(ps2x.buttonNewState(PSB_PAD_UP) && up == true){
      up = false;
  }

  // PAD DOWN BUTTON PRESS -> SET VOLTAGE LIMIT TO 1.9
  else if(ps2x.buttonNewState(PSB_PAD_DOWN) && down == false){
      Serial.println("Change voltage limit: 1.9.");
      if (jl){
        //sendData("MQL", 1.9, HT1);
        //sendData("MDL", 1.9, HT1);
        //sendData("MQL", 1.9, KT1);
        //sendData("MDL", 1.9, KT1);
      }
  }
  else if(ps2x.buttonNewState(PSB_PAD_DOWN) && down == true){
      down = false;
  }

  // PAD RIGHT BUTTON PRESS -> SET VOLTAGE LIMIT TO 2.5
  else if(ps2x.buttonNewState(PSB_PAD_RIGHT) && right == false){
      Serial.println("Change voltage limit: 2.2.");
      if (jl){
        //sendData("MQL", 2.5, HT1);
        //sendData("MDL", 2.5, HT1);
        //sendData("MQL", 2.5, KT1);
        //sendData("MDL", 2.5, KT1);
      }
  }
  else if(ps2x.buttonNewState(PSB_PAD_RIGHT) && right == true){
      right = false;
  }

  // CIRCLE BUTTON PRESS -> SET ACTUATORS TO POSITION CONTROL MODE
  if(ps2x.button(PSB_L2)){
    if(ps2x.buttonNewState(PSB_CIRCLE) && circle == false){
      Serial.println("DAGOR A: Position mode");
      circle = true;
      //Send C2
      //sendData("MC2", 0, HT1);
    }
    else if(ps2x.buttonNewState(PSB_CIRCLE) && circle == true){
      circle = false;
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == false){
      Serial.println("Request Position.");
      //sendData("pos", 0, HT1);
      //sendData("pos", 0, KT1);
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == true){
      cross = false;
    }
  }

  if(ps2x.button(PSB_R2)){
    if(ps2x.buttonNewState(PSB_CIRCLE) && circle == false){
      Serial.println("DAGOR B: Position mode");
      circle = true;
      target2 = 0;
      //Send target = 0
      //Send C2
      //sendData("MC2", 0, KT1);
    }
    else if(ps2x.buttonNewState(PSB_CIRCLE) && circle == true){
      circle = false;
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == false){
      Serial.println("Request Position.");
      //sendData("pos", 0, (uint8_t*)KT1);
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == true){
      cross = false;
    }
  }
}
*/
