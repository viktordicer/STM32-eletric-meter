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


*********************************/
// LIBRARY
#include <Arduino.h>
#include <Ethernet.h>  //ETHERNET LIBRARY
#include <PubSubClient.h> //MQTT LIBRARY
//#include <ssd1306.h>
//#include <nano_gfx.h>
#include "configuration.h"
#include "electric_phase.h"

//#define DEBUG

// Methodes
void ethernetReset();
void mqttConnection();
void L1_interr();
void L2_interr();
void L3_interr();
double power_W(uint32_t delta_phase_count);

void dataToChar();
void powerComputing();
void sendData();
void serialPrint(String message);

//------------------------- Program variables
//NEW Objects
IPAddress ip(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);                     //IP address of this device
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

ElectricPhase l1(INTERRUPT_L1);
ElectricPhase l2(INTERRUPT_L2);
ElectricPhase l3(INTERRUPT_L3);

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
  #ifdef DEBUG
    Serial.begin(57600);
    Serial.println("STM32 strarting...");
  #endif //DEBUG

  attachInterrupt(digitalPinToInterrupt(PB15),L1_interr, RISING); // interrupt for L1 phase
  attachInterrupt(digitalPinToInterrupt(PB14),L2_interr, RISING); // interrupt for L2 phase
  attachInterrupt(digitalPinToInterrupt(PB13),L3_interr, RISING); // interrupt for L3 phase

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
      serialPrint("MQTT connection is killed.");
      mqttConnection();
    }else{
      serialPrint("MQTT connection is still alive.");
    }
    sendData(); 
    mqttClient.loop();
  }
}
// Send all data to MQTT brocker
void sendData(){
  if(mqtt_conn){
    mqttClient.publish(L1_POWER_TOPIC, powerL1_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_POWER_TOPIC, powerL2_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L3_POWER_TOPIC, powerL3_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(FOYER_CONNECTION_FAILED, connection_failed_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L1_CONSUM_INC_TOPIC, consumL1_mqtt_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L2_CONSUM_INC_TOPIC, consumL2_mqtt_char);
    delay(MQTT_SENDING_DEALY);
    mqttClient.publish(L3_CONSUM_INC_TOPIC, consumL3_mqtt_char);
    consumL1_mqtt = 0;
    consumL2_mqtt = 0;
    consumL3_mqtt = 0;
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
        serialPrint("Can't connect to the mqtt.");
        return;
      }
      serialPrint("Attempting MQTT connection...");

      if (mqttClient.connect(MQTT_CLIENT_ID, USERNAME, PASSWORD )) {
        serialPrint("MQTT connected");
        mqtt_conn = true;
      } else {
        serialPrint(" try again in 2 seconds");
        delay(1000);
        connectionAtempt ++;
      }
    }
  }
  delay(200);
}

// Physically reset ethernet adapter. Need to call befor mqtt connection
void ethernetReset() {
  serialPrint("Reseting ethernet adapter");
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN,HIGH);
  delay(100);
  pinMode(RESET_PIN, INPUT);
  delay(100);
  serialPrint("Initialize ethernet");
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

// Get power and consumption of all phases.
void powerComputing(){
  powerL1 = l1.getPower(time - last_time_measure);
  powerL2 = l2.getPower(time - last_time_measure);
  powerL3 = l3.getPower(time - last_time_measure);

  consumptionL1_inc = l1.getConsumption();
  consumptionL2_inc = l2.getConsumption();
  consumptionL3_inc = l3.getConsumption();

  l1.clearCount();
  l2.clearCount();
  l3.clearCount();

  consumL1_mqtt += consumptionL1_inc;
  consumL2_mqtt += consumptionL2_inc;
  consumL3_mqtt += consumptionL3_inc;
}

// Convert variables to char for MQTT publisher
void dataToChar(){
  double consum_scale = 100.0; // scale for comsumption in kWh. Incrementation of consumption is too small number.

  dtostrf(powerL1, 5, 1, powerL1_char);
  dtostrf(powerL2, 5, 1, powerL2_char);
  dtostrf(powerL3, 5, 1, powerL3_char);
  dtostrf(consumL1_mqtt * consum_scale, 4, 3, consumL1_mqtt_char);
  dtostrf(consumL2_mqtt * consum_scale, 4, 3, consumL2_mqtt_char);
  dtostrf(consumL3_mqtt * consum_scale, 4, 3, consumL3_mqtt_char);
  sprintf(connection_failed_char, "%i", connection_failed);
}

// Serial print if debug mode is on
void serialPrint(String message){
  #ifdef DEBUG
    Serial.println(message);
  #endif
}