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
#include "configuration.h"
#include "electric_phase.h"

// Methodes
void ethernetReset();
void mqttConnection();
void L1_interr();
void L2_interr();
void L3_interr();
// appliances
void L1_oven_interr();
void L2_refrigerator_interr();
void L3_dishwasher_interr();

double power_W(uint32_t delta_phase_count);

void dataToChar();
void powerComputing();
void sendData();

//------------------------- Program variables
//NEW Objects
IPAddress ip(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);                     //IP address of this device
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

ElectricPhase l1(INTERRUPT_L1);
ElectricPhase l2(INTERRUPT_L2);
ElectricPhase l3(INTERRUPT_L3);

ElectricPhase l1_oven(INTERRUPT_L1_OVEN);
ElectricPhase l2_refrigerator(INTERRUPT_L2_REFRIGERATOR);
ElectricPhase l3_dishwasher(INTERRUPT_L3_DISHWASHER);

// VARIABLES

double powerL1 = 0.00; // power L1 phase in W
double powerL2 = 0.00; // power L2 phase in W
double powerL3 = 0.00; // power L3 phase in W

char powerL1_char[15]; //L1 char power
char powerL2_char[15]; //L2 char power
char powerL3_char[15]; //L3 char power

double consumptionL1_inc = 0.0000; // increment consumption L1 phase in kWh
double consumptionL2_inc = 0.0000; // increment consumption L2 phase in kWh
double consumptionL3_inc = 0.0000; // increment consumption L3 phase in kWh

double consumL1_mqtt = 0.0000; // increment consumption L1 phase in kWh
double consumL2_mqtt = 0.0000; // increment consumption L2 phase in kWh
double consumL3_mqtt = 0.0000; // increment consumption L3 phase in kWh

char consumL1_mqtt_char[15]; //L1 char consumption increment
char consumL2_mqtt_char[15]; //L2 char consumption increment
char consumL3_mqtt_char[15]; //L3 char consumption increment

// Appliances
double powerL1_oven = 0.00; // power L1 phase in W
double powerL2_refri = 0.00; // power L2 phase in W
double powerL3_dish = 0.00; // power L3 phase in W

char powerL1_oven_char[15]; 
char powerL2_refri_char[15]; 
char powerL3_dish_char[15]; 

double consumptionL1_oven_inc = 0.0000; // increment consumption L1 phase in kWh
double consumptionL2_refri_inc = 0.0000; // increment consumption L2 phase in kWh
double consumptionL3_dish_inc = 0.0000; // increment consumption L3 phase in kWh

double consumL1_oven_mqtt = 0.0000; 
double consumL2_refri_mqtt = 0.0000; 
double consumL3_dish_mqtt = 0.0000; 

char consumL1_oven_mqtt_char[15]; //L1 char consumption increment
char consumL2_refri_mqtt_char[15]; //L2 char consumption increment
char consumL3_dish_mqtt_char[15]; //L3 char consumption increment

// Failed
int connection_failed = 0;
bool mqtt_conn = false;
char connection_failed_char[15];

// OLED VARIABLES
int screen = 0;

//TIME VARIABLES
uint32_t time =  0;
uint32_t last_time_measure =  0;

// ------------------ SETUP ---------------------------------------------------------
void setup() {

  attachInterrupt(digitalPinToInterrupt(PB15),L1_interr, RISING); // interrupt for L1 phase
  attachInterrupt(digitalPinToInterrupt(PB14),L2_interr, RISING); // interrupt for L2 phase
  attachInterrupt(digitalPinToInterrupt(PB13),L3_interr, RISING); // interrupt for L3 phase

  attachInterrupt(digitalPinToInterrupt(PA12),L1_oven_interr, RISING); // interrupt for L1 phase
  attachInterrupt(digitalPinToInterrupt(PA11),L2_refrigerator_interr, RISING); // interrupt for L2 phase
  attachInterrupt(digitalPinToInterrupt(PB10),L3_dishwasher_interr, RISING); // interrupt for L3 phase

  Ethernet.begin(MAC);
  delay(1500);
  mqttClient.setServer(MQTT_SERVER_IP,1883);
}

// ------------------ LOOP ---------------------------------------------------------
void loop() {
  Ethernet.maintain();
  time = millis();

  if(time-last_time_measure > MEASURE_INTERVAL){
    powerComputing();
    last_time_measure = time;
    dataToChar(); // all data conversion to char for mqtt and screen

    //CHECK MQTT CONNECTION IS ALIVE
    if(!mqtt_conn){
      ethernetReset();
    }
    if(!mqttClient.connected()){
      mqttConnection();
    }
    sendData(); 
    mqttClient.loop();
  }
}
// Send all data to MQTT brocker
void sendData(){
  if(mqtt_conn){
    // MAIN POWER
    mqttClient.publish(L1_POWER_TOPIC, powerL1_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_POWER_TOPIC, powerL2_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L3_POWER_TOPIC, powerL3_char);
    delay(MQTT_SENDING_DEALY);

    // APPLIANCES POWER
    mqttClient.publish(L1_OVEN_TOPIC, powerL1_oven_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_REFRIGERATOR_TOPIC, powerL2_refri_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_DISHWASHER_TOPIC, powerL3_dish_char);
    delay(MQTT_SENDING_DEALY);

    // MAIN INCREMENT
    mqttClient.publish(L1_CONSUM_INC_TOPIC, consumL1_mqtt_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_CONSUM_INC_TOPIC, consumL2_mqtt_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L3_CONSUM_INC_TOPIC, consumL3_mqtt_char);
    delay(MQTT_SENDING_DEALY);

    // APPLIANCES INCREMENT
    mqttClient.publish(L1_OVEN_INC_TOPIC, consumL1_oven_mqtt_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_REFRIGERATOR_INC_TOPIC, consumL2_refri_mqtt_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_DISHWASHER_INC_TOPIC, consumL3_dish_mqtt_char);
    delay(MQTT_SENDING_DEALY);

    // FAILED
    mqttClient.publish(FOYER_CONNECTION_FAILED, connection_failed_char);
    delay(MQTT_SENDING_DEALY);
    consumL1_mqtt = 0;
    consumL2_mqtt = 0;
    consumL3_mqtt = 0;

    consumL1_oven_mqtt = 0;
    consumL2_refri_mqtt = 0;
    consumL3_dish_mqtt = 0;
  }
}
// *SENDING DATA -------------------------------------------------------
// Connect to MQTT brocker. When not available, try connect 3x
void mqttConnection() {
  int connectionAtempt = 1;
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
        mqtt_conn = true;
      } else {
        delay(1000);
        connectionAtempt ++;
      }
    }
  }
  delay(200);
}

// Physically reset ethernet adapter. Need to call befor mqtt connection
void ethernetReset() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN,HIGH);
  delay(100);
  pinMode(RESET_PIN, INPUT);
  delay(100);
  Ethernet.begin(MAC); //, ip);

  //delay with ethernet maintain
  for (int i = 0; i <= 10; i++) {
    delay(100);
    Ethernet.maintain();
  }
  mqttClient.setServer(MQTT_SERVER_IP,1883);
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

// Get power and consumption of all phases.
void powerComputing(){
  uint32_t delta_time = time - last_time_measure;
  //MAIN POWER
  powerL1 = l1.getPower(delta_time);
  powerL2 = l2.getPower(delta_time);
  powerL3 = l3.getPower(delta_time);

  //APPLIANCES POWER
  powerL1_oven = l1_oven.getPower(delta_time);
  powerL2_refri = l2_refrigerator.getPower(delta_time);
  powerL3_dish= l3_dishwasher.getPower(delta_time);

  // MAIN INCREMENT
  consumptionL1_inc = l1.getConsumption();
  consumptionL2_inc = l2.getConsumption();
  consumptionL3_inc = l3.getConsumption();

  consumptionL1_oven_inc = l1_oven.getConsumption();
  consumptionL2_refri_inc = l2_refrigerator.getConsumption();
  consumptionL3_dish_inc = l3_dishwasher.getConsumption();

  l1.clearCount();
  l2.clearCount();
  l3.clearCount();
  l1_oven.clearCount();
  l2_refrigerator.clearCount();
  l3_dishwasher.clearCount();


  consumL1_mqtt += consumptionL1_inc;
  consumL2_mqtt += consumptionL2_inc;
  consumL3_mqtt += consumptionL3_inc;

  consumL1_oven_mqtt += consumptionL1_oven_inc;
  consumL2_refri_mqtt += consumptionL2_refri_inc;
  consumL3_dish_mqtt += consumptionL3_dish_inc;
}

// Convert variables to char for MQTT publisher
void dataToChar(){
  double consum_scale = 100.0; // scale for comsumption in kWh. Incrementation of consumption is too small number.
  //MAIN POWER
  dtostrf(powerL1, 5, 1, powerL1_char);
  dtostrf(powerL2, 5, 1, powerL2_char);
  dtostrf(powerL3, 5, 1, powerL3_char);
  //APPLIANCES POWER
  dtostrf(powerL1_oven, 5, 1, powerL1_oven_char);
  dtostrf(powerL2_refri, 5, 1, powerL2_refri_char);
  dtostrf(powerL3_dish, 5, 1, powerL3_dish_char);

  //MAIN INCREMENT
  dtostrf(consumL1_mqtt * consum_scale, 4, 3, consumL1_mqtt_char);
  dtostrf(consumL2_mqtt * consum_scale, 4, 3, consumL2_mqtt_char);
  dtostrf(consumL3_mqtt * consum_scale, 4, 3, consumL3_mqtt_char);

  //APPLIANCES INCREMENT
  dtostrf(consumL1_oven_mqtt * consum_scale, 4, 3, consumL1_oven_mqtt_char);
  dtostrf(consumL2_refri_mqtt * consum_scale, 4, 3, consumL2_refri_mqtt_char);
  dtostrf(consumL3_dish_mqtt * consum_scale, 4, 3, consumL3_dish_mqtt_char);

  sprintf(connection_failed_char, "%i", connection_failed);
}
