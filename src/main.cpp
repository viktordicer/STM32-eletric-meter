/********************************
  W5500 TO STM32 TYPICAL WIRING
    VCC ----> 5V
    GND ----> GND
    CS  ----> PA4
    SCK ----> PA5
    SO  ----> PA6
    SI  ----> PA7
    RST ----> PA0

  Phase interrupts
    l1 ----> PB15
    l2 ----> PB14
    l3 ----> PB13

    L1_oven  ----> PA12
    L2_refri ----> PA11
    L3_dish  ----> PA10

*********************************/
// LIBRARY
#include <Arduino.h>
#include <Ethernet.h>  //ETHERNET LIBRARY
#include <PubSubClient.h> //MQTT LIBRARY
#include <ArduinoJson.h>
#include "configuration.h"
#include "electric_phase.h"
#include <math.h>

#define DEBUG

#ifdef DEBUG
  #define logln(x) Serial.println(x)
#elif
  #define logln(x)
#endif

// Methodes
void ethernetReset();
void mqttConnection();
void callback(char* topic, byte* payload, unsigned int length);
void L1_interr();
void L2_interr();
void L3_interr();
// appliances
void L1_oven_interr();
void L2_refrigerator_interr();
void L3_dishwasher_interr();

void sendData();

//------------------------- Program variables
//NEW Objects
IPAddress ip(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);                     //IP address of this device
EthernetClient ethClient;
PubSubClient mqttClient(MQTT_SERVER_IP, 1883, callback, ethClient);

ElectricPhase l1(INTERRUPT_L1);
ElectricPhase l2(INTERRUPT_L2);
ElectricPhase l3(INTERRUPT_L3);

ElectricPhase l1_oven(INTERRUPT_L1_OVEN);
ElectricPhase l2_refrigerator(INTERRUPT_L2_REFRIGERATOR);
ElectricPhase l3_dishwasher(INTERRUPT_L3_DISHWASHER);

// Failed
int connection_failed = 0;
bool mqtt_conn = false;
char connection_failed_char[15];

//TIME VARIABLES
uint32_t time =  0;
uint32_t last_time_measure =  0;
uint32_t measure_interval  =  120000; // delay in ms

// ------------------ SETUP ---------------------------------------------------------
void setup() {
#ifdef DEBUG
  Serial.begin(57600);
#endif
  attachInterrupt(digitalPinToInterrupt(PB15),L1_interr, FALLING); // interrupt for L1 phase
  attachInterrupt(digitalPinToInterrupt(PB14),L2_interr, FALLING); // interrupt for L2 phase
  attachInterrupt(digitalPinToInterrupt(PB13),L3_interr, FALLING); // interrupt for L3 phase

  attachInterrupt(digitalPinToInterrupt(PA12),L1_oven_interr, FALLING); // interrupt for L1 phase
  attachInterrupt(digitalPinToInterrupt(PA11),L2_refrigerator_interr, FALLING); // interrupt for L2 phase
  attachInterrupt(digitalPinToInterrupt(PA9),L3_dishwasher_interr, FALLING); // interrupt for L3 phase

  Ethernet.begin(MAC);
  delay(1500);
  mqttClient.setBufferSize(512);
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  mqttClient.setCallback(callback);
  mqttConnection();
  logln("STM32 starting ......");
}

// ------------------ LOOP ---------------------------------------------------------
void loop() {
  Ethernet.maintain();
  
  if(millis()-last_time_measure > measure_interval){
    //CHECK MQTT CONNECTION IS ALIVE
    if(!mqttClient.connected()){
      mqttConnection();
    }else{
      sendData(); 
    }
  }
  mqttClient.loop();
}

//MQTT - send data
void sendData(){
  time = millis();
  uint32_t delta_time = time - last_time_measure;
  StaticJsonDocument<512> payload;
  char buffer[512];

  payload["pL1"] = int(l1.getPower(delta_time));
  payload["pL2"] = int(l2.getPower(delta_time));
  payload["pL3"] = int(l3.getPower(delta_time));

  //APPLIANCES POWER
  payload["pL1_oven"] = int(l1_oven.getPower(delta_time));
  payload["pL2_refri"] = int(l2_refrigerator.getPower(delta_time));
  payload["pL3_dish"] = int(l3_dishwasher.getPower(delta_time));

  // MAIN INCREMENT
  payload["L1_inc"] = l1.getConsumptionInctement();
  payload["L2_inc"] = l2.getConsumptionInctement();
  payload["L3_inc"] = l3.getConsumptionInctement();

  payload["conOven"] = l1_oven.getConsumptionInctement();
  payload["conRefri"] = l2_refrigerator.getConsumptionInctement();
  payload["conDish"] = l3_dishwasher.getConsumptionInctement();
  //TOTAL CONSUMPTION
  payload["tL1"] = round(l1.getTotalConsumption()*10)/10;
  payload["tL2"] = round(l2.getTotalConsumption()*10)/10;
  payload["tL3"] = round(l3.getTotalConsumption()*10)/10;

  payload["tOven"] = round(l1_oven.getTotalConsumption()*10)/10;
  payload["tRefri"] = round(l2_refrigerator.getTotalConsumption()*10)/10;
  payload["tDish"] = round(l3_dishwasher.getTotalConsumption()*10)/10;
  payload["con_failed"] = connection_failed;

  serializeJson(payload, buffer);
  mqttClient.publish(POWER_TOPIC, buffer);
  
  logln("Data sent.");
  l1.clearCount();
  l2.clearCount();
  l3.clearCount();
  l1_oven.clearCount();
  l2_refrigerator.clearCount();
  l3_dishwasher.clearCount();
  last_time_measure = time;
  
}

// *SENDING DATA -------------------------------------------------------
// Connect to MQTT brocker. When not available, try connect 3x
void mqttConnection() {
  int connectionAtempt = 1;
  logln("MQTT conecting .....");
  if(!mqttClient.connected()){
    while (!mqttClient.connected()) {
      if(connectionAtempt > 3){
        mqttClient.disconnect();
        delay(300);
        ethernetReset();
        connection_failed ++;
        mqtt_conn = false;
        return;
      }
      if (mqttClient.connect(MQTT_CLIENT_ID, USERNAME, PASSWORD )) {
        mqttClient.subscribe(SET_L1_TOTAL_TOPIC);
        mqttClient.subscribe(SET_L2_TOTAL_TOPIC);
        mqttClient.subscribe(SET_L3_TOTAL_TOPIC);
        mqttClient.subscribe(SET_OVEN_TOTAL_TOPIC);
        mqttClient.subscribe(SET_REFRI_TOTAL_TOPIC);
        mqttClient.subscribe(SET_DISH_TOTAL_TOPIC);
        mqttClient.subscribe(HA_TOPIC);
        mqttClient.subscribe(SET_INTERVAL_TOPIC);
        mqttClient.publish(STATUS_TOPIC, "online");
        mqtt_conn = true;
        logln("MQTT connected");
      } else {
        delay(1000);
        connectionAtempt ++;
        logln("MQTT failed");
      }
    }
  }
  delay(200);
}

// Physically reset ethernet adapter. Need to call befor mqtt connection
void ethernetReset() {
  logln("Ethernet resetting...");
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN,HIGH);
  delay(100);
  pinMode(RESET_PIN, INPUT);
  delay(100);
  Ethernet.begin(MAC); //, ip);
  delay(1000);
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  mqttClient.setCallback(callback);
}

// *FLOW COMPUTING -------------------------------------------------------
// increment L1 phase pulses
void L1_interr(){
  l1.addCount();
}
// increment L2 phase pulses
void L2_interr(){
  l2.addCount();
}
// increment L3 phase pulses
void L3_interr(){
  l3.addCount();
}
// increment L1 oven phase pulses
void L1_oven_interr(){
  l1_oven.addCount();
}
// increment L2 refrigerator phase pulses
void L2_refrigerator_interr(){
  l2_refrigerator.addCount();
}
// increment L3 dishwasheer phase pulses
void L3_dishwasher_interr(){
  l3_dishwasher.addCount();
}


// *MQTT CALLBACK 
void callback(char* topic, byte* payload, unsigned int length) {
  byte* p = (byte*)malloc(length);
  //Copy the payload to the new buffer
  memcpy(p,payload,length);

  String msg;
  for(int i=0; i<length; i++){
    msg += (char)payload[i]; 
  }
  
  if(strcmp(topic,HA_TOPIC)==0){
    if(msg == "online"){
      //send data
      sendData();
    }
  } else if(strcmp(topic,SET_L1_TOTAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      l1.setTotalConsumption(s.toDouble());
  } else if(strcmp(topic,SET_L2_TOTAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      l2.setTotalConsumption(s.toDouble());
  } else if(strcmp(topic,SET_L3_TOTAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      l3.setTotalConsumption(s.toDouble());
  } else if(strcmp(topic,SET_OVEN_TOTAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      l1_oven.setTotalConsumption(s.toDouble());
  } else if(strcmp(topic,SET_REFRI_TOTAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      l2_refrigerator.setTotalConsumption(s.toDouble());
  } else if(strcmp(topic,SET_DISH_TOTAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      l3_dishwasher.setTotalConsumption(s.toDouble());
  } else if(strcmp(topic,SET_INTERVAL_TOPIC)==0){
      payload[length] = '\0';
      String s = String((char*)payload);
      measure_interval = s.toDouble();
  }
  free(p);
}