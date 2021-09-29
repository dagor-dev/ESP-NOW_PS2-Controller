//Sparkfun: 24:6F:28:51:ED:A4
//Test Station 1: AC:67:B2:59:F9:9C
//Test Station 2: AC:67:B2:5B:0B:54  (without regulator)

//LEG 1: AC:67:B2:59:AB:B8
//LEG 2: AC:67:B2:5A:FE:D8 (RIP)
//LEG 2: AC:67:B2:5A:7B:3C

#include "PsxLib.h"
#include <esp_now.h>
#include <WiFi.h>

Psx ps2x; // create PS2 Controller Class

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress1[] = {0xAC, 0x67, 0xB2, 0x59, 0xF9, 0x9C};

uint8_t broadcastAddress1[] = {0xAC, 0x67, 0xB2, 0x5A, 0x7B, 0x3C};       //Actuator A
uint8_t broadcastAddress2[] = {0xAC, 0x67, 0xB2, 0x59, 0xAB, 0xB8};       //Actuator B

// Structure example to send data (Must match the receiver structure)
typedef struct struct_message {
  String function;
  float value;
} struct_message;

// Create a struct_message
struct_message dagorData;
struct_message inputData;
esp_now_peer_info_t peerInfo;

float target = 0, target2 = 0;
float a0, a1, a2, a3;
float multiplier = 0.01, constant = 0.01, multiplier2 = 0.01;
float adder = 0, adder2 = 0;
bool triangle = true, circle = true, square = true, cross = true;

//####_DEMO Arrays_#####
//Jumping sequence
float jumpA[] = {  1.74,  5.32,  -0.18,   1.74};
float jumpB[] = {-12.71, -6.82, -16.21, -12.71};
int sp = 0;
bool jumping = false;

//#####_TIME MANAGEMENT_#####
float runTime, prevT = 0, timeDif, stateT;
int timeInterval = 1000, totalTempTime;

void setup(){
  Serial.begin(115200);
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  //error = ps2x.config_gamepad(15, 4, 14, 12, true, true);
  ps2x.setupPins(26, 27, 25, 32, 50);

  delay(300);

  espNowInit();
}

void loop() {
  buttonPress();

  //Time managment for DEMOs' movements
  runTime = micros();
  timeDif = runTime - prevT;
  prevT = runTime;
  stateT += timeDif;

  if(stateT >= 500000/2){
    stateT = 0;
    
    if(jumping == true){
      sendData1("Target",jumpA[sp]);
      sendData2("Target",jumpB[sp]);
      sp += 1;
    }

    if(sp == 4){
      sp = 0;
      jumping = false;
    }
    
  }
   
}

void buttonPress(){
  unsigned long datos = ps2x.read();
  
  if(ps2x.button(PSB_L1)){
    a2 = ps2x.analog(2);
    a3 = ps2x.analog(3);
    multiplier += ((a2-128)/128) * constant;
    if (multiplier <= 0) multiplier = 0;
    adder = -((a3-128)/128);
    target += adder*multiplier;
    Serial.print(adder);
    Serial.print(", ");
    Serial.print(multiplier);
    Serial.print(", ");
    Serial.println(target);
    
    //Send target with ESPNOW
    sendData1("Target",target);
    delay(40); 
  }

  if(ps2x.button(PSB_R1)){
    a0 = ps2x.analog(0);
    a1 = ps2x.analog(1);
    multiplier2 += ((a0-128)/128) * constant;
    if (multiplier2 <= 0) multiplier2 = 0;
    adder2 = -((a1-128)/128);
    target2 += adder2*multiplier2;
    Serial.print(adder2);
    Serial.print(", ");
    Serial.print(multiplier2);
    Serial.print(", ");
    Serial.println(target2);
    
    //Send target with ESPNOW
    sendData2("Target",target2);
    delay(40); 
  }
  
  if(ps2x.button(PSB_L2)){
    if(ps2x.buttonNewState(PSB_TRIANGLE) && triangle == false){
      Serial.println("Jumping start");
      jumping = true;
    }
    else if(ps2x.buttonNewState(PSB_TRIANGLE) && triangle == true){
      triangle = false;
    }
    else if(ps2x.buttonNewState(PSB_CIRCLE) && circle == false){
      Serial.println("DAGOR A: Position mode");
      circle = true;
      target = 0;
      //Send target = 0
      //Send C2
      sendData1("C2", 0);
    }
    else if(ps2x.buttonNewState(PSB_CIRCLE) && circle == true){
      circle = false;
    }
    else if(ps2x.buttonNewState(PSB_SQUARE) && square == false){
      Serial.println("DAGOR A: Voltage mode");
      square = true;
      target = 0;
      multiplier = 0.01;
      //Send target = 0
      //Send C0
      sendData1("C0", 0);
    }
    else if(ps2x.buttonNewState(PSB_SQUARE) && square == true){
      square = false;
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == false){
      Serial.println("Request Position.");
      sendData1("pos", 0);
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == true){
      cross = false;
    }
  }

  if(ps2x.button(PSB_R2)){
    if(ps2x.buttonNewState(PSB_TRIANGLE) && triangle == false){
      Serial.println("DAGOR B: Velocity mode");
      triangle = true;
      target2 = 0;
      //Send target = 0
      //Send C1
      sendData2("C1", 0);
    }
    else if(ps2x.buttonNewState(PSB_TRIANGLE) && triangle == true){
      triangle = false;
    }
    else if(ps2x.buttonNewState(PSB_CIRCLE) && circle == false){
      Serial.println("DAGOR B: Position mode");
      circle = true;
      target2 = 0;
      //Send target = 0
      //Send C2
      sendData2("C2", 0);
    }
    else if(ps2x.buttonNewState(PSB_CIRCLE) && circle == true){
      circle = false;
    }
    else if(ps2x.buttonNewState(PSB_SQUARE) && square == false){
      Serial.println("DAGOR B: Voltage mode");
      square = true;
      target2 = 0;
      multiplier2 = 0.01;
      //Send target = 0
      //Send C0
      sendData2("C0", 0);
    }
    else if(ps2x.buttonNewState(PSB_SQUARE) && square == true){
      square = false;
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == false){
      Serial.println("Request Position.");
      sendData2("pos", 0);
    }
    else if(ps2x.buttonNewState(PSB_CROSS) && cross == true){
      cross = false;
    }
  }
}

void sendData1(String func, float val){
  dagorData.function = func;
  dagorData.value = val;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &dagorData, sizeof(dagorData));
  //if (result == ESP_OK) Serial.println("Sent with success");
  //else Serial.println("Error sending the data");
}

void sendData2(String func, float val){
  dagorData.function = func;
  dagorData.value = val;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress2, (uint8_t *) &dagorData, sizeof(dagorData));
  //if (result == ESP_OK) Serial.println("Sent with success");
  //else Serial.println("Error sending the data");
}

void espNowInit(){
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("Controller MAC address: ");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer 
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 

  // Add peer 0 
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Add peer 1
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nSend Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&inputData, incomingData, sizeof(inputData));
  Serial.print("Actuator: ");
  Serial.println(inputData.function);
  Serial.print("Position: ");
  Serial.println(inputData.value);

}
