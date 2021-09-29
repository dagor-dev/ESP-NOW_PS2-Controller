//##########################_ESP-NOW_##########################

#define TRY_ESP_ACTION(action, name) if(action == ESP_OK) {Serial.println("\t+ "+String(name));} else {Serial.println("----------Error while " + String(name) + " !---------------");}

#define CHANNEL 1
#define DATARATE WIFI_PHY_RATE_1M_L

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure to send data (Must match the receiver structure)
typedef struct output_message {
    int leg_id;
    String kneeFunc;
    float kneeValue;
    String hipFunc;
    float hipValue;
    String shoulderFunc;
    float shoulderValue;
} output_message;


// Create a struct_message
output_message outputData;
//struct_message inputData;
esp_now_peer_info_t peerInfo;

void sendData(int leg_id, String kneeFunc, float kneeValue, String hipFunc, float hipValue, String shoulderFunc, float shoulderValue){
  outputData.leg_id        = leg_id;
  outputData.kneeFunc      = kneeFunc;
  outputData.kneeValue     = kneeValue;
  outputData.hipFunc       = hipFunc;
  outputData.hipValue      = hipValue;
  outputData.shoulderFunc  = shoulderFunc;
  outputData.shoulderValue = shoulderValue;
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outputData, sizeof(outputData));     // Send message via ESP-NOW
  //if (result != ESP_OK) Serial.println("Error sending the data");
}

void espNowInit(){
  WiFi.disconnect();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("Controller MAC address: ");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  TRY_ESP_ACTION( esp_wifi_stop(), "Stop WIFI");
  TRY_ESP_ACTION( esp_wifi_deinit(), "ESP-NOW De-init");
  wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
  my_config.ampdu_tx_enable = 0;
  TRY_ESP_ACTION( esp_wifi_init(&my_config), "Disable AMPDU");
  TRY_ESP_ACTION( esp_wifi_start(), "Restart WiFi");
  TRY_ESP_ACTION( esp_wifi_set_promiscuous(true), "Set promiscuous");
  TRY_ESP_ACTION( esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE), "Channel set");
  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, DATARATE), "Fixed rate set up");
  TRY_ESP_ACTION( esp_now_init(), "ESP-NOW Init");

  esp_now_register_recv_cb(OnDataRecv);
  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);


  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); 
  peerInfo.channel = CHANNEL;  
  peerInfo.encrypt = false; 

  // Add peer
  TRY_ESP_ACTION( esp_now_add_peer(&peerInfo), "Add peer");

  /*
  // PEARS JUMPING LEG
  if (jl){
    memcpy(peerInfo.peer_addr, HT1, 6);      
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: ");
      //Serial.println(HT1);
      return;
    }
    memcpy(peerInfo.peer_addr, KT1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: ");
      //Serial.println(KT1);
      return;
    }    
  }

  // PEARS LEFT FRONT LEG
  if (tt){
    memcpy(peerInfo.peer_addr, TT1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: ");
      //Serial.println(TT1);
      return;
    }
  }


  // PEARS RIGHT FRONT LEG
  if (frl){
    memcpy(peerInfo.peer_addr, H1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: H1");
      return;
    }
    memcpy(peerInfo.peer_addr, S1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: S1");
      return;
    }
    memcpy(peerInfo.peer_addr, K1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: K1");
      return;
    }
  }


  // PEARS LEFT FRONT LEG
  if (fll){
    memcpy(peerInfo.peer_addr, H2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: H2");
      return;
    }
    memcpy(peerInfo.peer_addr, S2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: S2");
      return;
    }
    memcpy(peerInfo.peer_addr, K2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: K2");
      return;
    }    
  }

  
  // PEARS RIGHT BACK LEG
  if (brl){
    memcpy(peerInfo.peer_addr, H3, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: H3");
      return;
    }
    memcpy(peerInfo.peer_addr, S3, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: S3");
      return;
    }
    memcpy(peerInfo.peer_addr, K3, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: K3");
      return;
    }    
  }
  
  // PEARS LEFT BACK LEG
  if (bll){
    memcpy(peerInfo.peer_addr, H4, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: H4");
      //Serial.println(H4);
      return;
    }
    memcpy(peerInfo.peer_addr, S4, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: S4");
      //Serial.println(S4);
      return;
    }
    memcpy(peerInfo.peer_addr, K4, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Failed to add peer: K4");
      //Serial.println(K4);
      return;
    }    
  }
  */
}

void espNowDeinit(){
  Serial.println("Turning off ESP-NOW");
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error de-initializing ESP-NOW");
    return;
  }
  Serial.println("Turning off WiFi");
  WiFi.mode(WIFI_OFF);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\rStat:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "S" : "F");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  /*memcpy(&inputData, incomingData, sizeof(inputData));
  Serial.print("Actuator: ");
  Serial.println(inputData.function);
  Serial.print("Position: ");
  Serial.println(inputData.value);

  if(inputData.function = "A"){
    //correctA = inputData.value;
  }
  else if(inputData.function = "B"){
    //correctB = inputData.value;
  }
*/
}
