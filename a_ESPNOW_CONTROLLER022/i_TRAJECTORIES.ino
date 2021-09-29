//####_DEMO Arrays_#####


//Jumping sequence
float jumpA[] = {   5.66,   7.79,   8.30,   8.30,   1.86,   5.66,   5.66};
float jumpB[] = {  14.30,   8.53,   5.57,   5.57,  22.45,  14.30,  14.30};

float zJump[] = { 240, 170, 100, 100, 310, 240, 240};


void jump(){
  
  if(jumping == true){

      //z ( x ( zJump[sp], 70 ) );
    
      if(sp == 0){
        //sendData("MAP",25,HT1);
        //sendData("MAP",25,KT1);
      }
      else if(sp == 6){
        //sendData("MAP",3,HT1);
        //sendData("MAP",3,KT1);
      }
      //sendData("M",-jumpA[sp],HT1);
      //sendData("M",jumpB[sp],KT1);
      
      //sendData("M", legs[0].theta, HT1);
      //sendData("M", legs[0].phi,   KT1); 
      
      sp += 1;
    }

  if(sp == 7){
    sp = 0;
    jumping = false;
  }
}



void torqueTesting(){
  Serial.println("Starting torque test.");
  /*
  sendData("MC0",0, TT1);
  sendData("MQL3",0, TT1);
  sendData("MDL3",0, TT1);
  
  sendData("M",0, TT1);
    delay(1000);
  sendData("M", -2, TT1);
    delay(2000);
  sendData("M",0, TT1);
    delay(4000);
  sendData("M", -4, TT1);
    delay(2000);
  sendData("M",0, TT1);
    delay(4000);
  sendData("M", -6, TT1);
    delay(2000);
  sendData("M", 0, TT1);
    delay(4000);
  sendData("M", -8, TT1);
    delay(2000);
  sendData("M", 0, TT1);
    delay(4000);
  sendData("M", -10, TT1);
    delay(2000);
  sendData("M", 0, TT1);
    delay(4000);
  sendData("M", -12, TT1);
    delay(1500);
  sendData("M", 0, TT1);
    delay(4000);
  sendData("M", -14, TT1);
    delay(1500);
  sendData("M", 0, TT1);
    delay(4000);
  sendData("M", -16, TT1);
    delay(1500);
  sendData("M", 0, TT1);
    delay(4000);
  sendData("M", 0, TT1);
    torqueTest = false;
  */
  Serial.println("Torque test finished.");
}
