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
#include <ssd1306.h>
#include <nano_gfx.h>
#include "configuration.h"
#include "electric_phase.h"

//#define DEBUG
//#define DISPLAY


// Methodes
void ethernetReset();
void ethernetTurnoff();
void mqttConnection();
void L1_interr();
void L2_interr();
void L3_interr();
double power_W(uint32_t delta_phase_count);

void dataToChar();
void countScreen();
void powerScreen();
void consumptionScreen();
void powerSolve();

void serialPrint();


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

double consumptionL1 = 0.000; // consumption L1 phase in kWh
double consumptionL2 = 0.000; // consumption L2 phase in kWh
double consumptionL3 = 0.000; // consumption L3 phase in kWh

double consumptionL1_inc = 0.0000; // increment consumption L1 phase in kWh
double consumptionL2_inc = 0.0000; // increment consumption L2 phase in kWh
double consumptionL3_inc = 0.0000; // increment consumption L3 phase in kWh

double consumL1_mqtt = 0.0000; // increment consumption L1 phase in kWh
double consumL2_mqtt = 0.0000; // increment consumption L2 phase in kWh
double consumL3_mqtt = 0.0000; // increment consumption L3 phase in kWh

char consumptionL1_char[15]; //L1 char consumption
char consumptionL2_char[15]; //L2 char consumption
char consumptionL3_char[15]; //L3 char consumption

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
uint32_t last_time_screen =  0;
uint32_t last_consum_mqtt =  0;



// ------------------ SETUP ---------------------------------------------------------
void setup() {
  #ifdef DEBUG
    Serial.begin(57600);
    Serial.println("STM32 strarting...");
  #endif //DEBUG

  attachInterrupt(digitalPinToInterrupt(PB15),L1_interr, RISING); // interrupt for L1 phase
  attachInterrupt(digitalPinToInterrupt(PB14),L2_interr, RISING); // interrupt for L2 phase
  attachInterrupt(digitalPinToInterrupt(PB13),L3_interr, RISING); // interrupt for L3 phase

  #ifdef DISPLAY
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_128x64_i2c_init();
    ssd1306_clearScreen();
  #endif //DISPLAY

  Ethernet.begin(MAC);
  delay(1000);
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  //Serial.println("STM32 is ready");
}

// ------------------ LOOP ---------------------------------------------------------
void loop() {
  Ethernet.maintain();

  time = millis();

  #ifdef DISPLAY
    if(time-last_time_screen > SCREEN_INTERVAL){
      //OLED DISPLAY
      switch(screen){
        case 0:
          countScreen();
          screen = 1;
          break;
        case 1:
          powerScreen();
          screen = 2;
          break;
        case 2:
          consumptionScreen();
          screen = 0;
          break;
      }
      last_time_screen = time;
    }
  #endif //DISPLAY

  if(time-last_time_measure > MEASURE_INTERVAL){

    powerSolve();

    last_time_measure = time;
    dataToChar(); // all data conversion to char for mqtt and screen
  //CHECK MQTT CONNECTION IS ALIVE
  
    Ethernet.maintain();
    if(!mqttClient.connected()){
        //Serial.println("Mqtt connection is killed.");
        mqtt_conn = false;
        mqttConnection();
      }else{
        //Serial.println("Mqtt connection is allive.");
      }
    mqttClient.loop();

    if(mqtt_conn){
      mqttClient.publish(L1_POWER_TOPIC, powerL1_char);
      delay(MQTT_SENDING_DEALY);

      mqttClient.publish(L2_POWER_TOPIC, powerL2_char);
      delay(MQTT_SENDING_DEALY);

      mqttClient.publish(L3_POWER_TOPIC, powerL3_char);
      delay(MQTT_SENDING_DEALY);

      mqttClient.publish(L1_CONSUM_TOPIC, consumptionL1_char);
      delay(MQTT_SENDING_DEALY);

      mqttClient.publish(L2_CONSUM_TOPIC, consumptionL2_char);
      delay(MQTT_SENDING_DEALY);

      mqttClient.publish(L3_CONSUM_TOPIC, consumptionL3_char);
      delay(MQTT_SENDING_DEALY);

      mqttClient.publish(FOYER_CONNECTION_FAILED, connection_failed_char);
      delay(MQTT_SENDING_DEALY);
      
      if(time - last_consum_mqtt > CONSUMPTION_INTERVAL){

        mqttClient.publish(L1_CONSUM_INC_TOPIC, consumL1_mqtt_char);
        delay(MQTT_SENDING_DEALY);
      
        mqttClient.publish(L2_CONSUM_INC_TOPIC, consumL2_mqtt_char);
        delay(MQTT_SENDING_DEALY);
  
        mqttClient.publish(L3_CONSUM_INC_TOPIC, consumL3_mqtt_char);

        consumL1_mqtt = 0;
        consumL2_mqtt = 0;
        consumL3_mqtt = 0;

        last_consum_mqtt = time;
      }
    }
  }
  
}

// *SENDING DATA -------------------------------------------------------

void mqttConnection() {
  //ethernetReset();
  // setup mqtt clients
  int connectionAtempt = 1;
  if(!mqttClient.connected()){
    while (!mqttClient.connected()) {
      //after 2 atempts reset W5500
      if(connectionAtempt > 3){
        mqttClient.disconnect();
        //ethClient.stop();
        ethernetReset();//ethernetTurnoff();
        connection_failed ++;
        mqtt_conn = false;
        return;
      }
      //Serial.println("Attempting MQTT connection...");
      // Attempt to connect
/*
      String clientId = "Foyer_el_room-";
      clientId += String(random(0xffff), HEX);
      clientId.c_str()*/

      if (mqttClient.connect(MQTT_CLIENT_ID, USERNAME, PASSWORD )) {
        //Serial.println("connected");
        //mqttClient.subscribe(SUBSCRIBE_TOPIC);
        mqtt_conn = true;
      } else {
        //Serial.println("failed, rc=");
        //Serial.println(mqttClient.state());
        //Serial.println(" try again in 1 seconds");
        delay(1000);
        //Serial.print("Connection atempt: "); 
        //Serial.println(connectionAtempt);
        connectionAtempt ++;
        //ethernetReset();
      }
    }
  }
  delay(200);
  //Serial.println("Connected to the mqtt");
}

void ethernetReset() {
  //Define reset pin for W5
  //ethClient.stop();
  //Serial.println("Reseting ethernet adapter");
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN,HIGH);
  delay(100);
  pinMode(RESET_PIN, INPUT);
  delay(100);
  //Serial.println("Initialize ethernet");
  Ethernet.begin(MAC); //, ip);

  //delay with ethernet maintain
  for (int i = 0; i <= 8; i++) {
    delay(100);
    Ethernet.maintain();
  }
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  //mqttClient.setCallback(callback);
}



void serialPrint(){
  /*Serial.print("L1 power in W: ");
  Serial.println(powerL1);
  Serial.print("L2 power in W ");
  Serial.println(powerL2);
  Serial.print("L3 power in W: ");
  Serial.print("L1 consumption in kWh: ");
  Serial.println(consumptionL1);
  Serial.print("L2 consumption in kWh ");
  Serial.println(consumptionL2);
  Serial.print("L3 consumption in kWh: ");
  Serial.println(consumptionL3);*/

}

// *FLOW COMPUTING -------------------------------------------------------
// increment L1 phase pulses
void L1_interr() {
  l1.addCount() ;
}
// increment L2 phase pulses
void L2_interr() {
  l2.addCount();
}
// increment L3 phase pulses
void L3_interr() {
  l3.addCount();
}


void powerSolve(){
  powerL1 = l1.getPower(time - last_time_measure);
  powerL2 = l2.getPower(time - last_time_measure);
  powerL3 = l3.getPower(time - last_time_measure);

  consumptionL1_inc = l1.getConsumption();
  consumptionL2_inc = l2.getConsumption();
  consumptionL3_inc = l3.getConsumption();

  consumptionL1 += consumptionL1_inc;
  consumptionL2 += consumptionL2_inc;
  consumptionL3 += consumptionL3_inc;

  consumL1_mqtt += consumptionL1_inc;
  consumL2_mqtt += consumptionL2_inc;
  consumL3_mqtt += consumptionL3_inc;
}


void dataToChar(){
  double consum_scale = 100.0;

  dtostrf(powerL1, 5, 1, powerL1_char);
  dtostrf(powerL2, 5, 1, powerL2_char);
  dtostrf(powerL3, 5, 1, powerL3_char);

  dtostrf(consumptionL1, 8, 3, consumptionL1_char);
  dtostrf(consumptionL2, 8, 3, consumptionL2_char);
  dtostrf(consumptionL3, 8, 3, consumptionL3_char);

  dtostrf(consumL1_mqtt * consum_scale, 4, 3, consumL1_mqtt_char);
  dtostrf(consumL2_mqtt * consum_scale, 4, 3, consumL2_mqtt_char);
  dtostrf(consumL3_mqtt * consum_scale, 4, 3, consumL3_mqtt_char);

  sprintf(connection_failed_char, "%i", connection_failed);
}



// * ------------ DISPLAY SCREEN  ----------------

void countScreen(){
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_clearScreen();
  ssd1306_printFixed(0,  8, "L1 cnt: ", STYLE_NORMAL);
  ssd1306_printFixed(50, 8, L1_count_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  24, "L2 cnt: ", STYLE_NORMAL);
  ssd1306_printFixed(50, 24, L2_count_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  40, "L3 cnt: ", STYLE_NORMAL);
  ssd1306_printFixed(50, 40, L3_count_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  56, "Conn failed: ", STYLE_NORMAL);
  ssd1306_printFixed(90, 56, connection_failed_char, STYLE_NORMAL);
}

void powerScreen(){
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_clearScreen();
  ssd1306_printFixed(0,  8, "L1 power: ", STYLE_NORMAL);
  ssd1306_printFixed(60, 8, powerL1_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  24, "L2 power: ", STYLE_NORMAL);
  ssd1306_printFixed(60, 24, powerL2_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  40, "L3 power: ", STYLE_NORMAL);
  ssd1306_printFixed(60, 40, powerL3_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  56, "Conn failed: ", STYLE_NORMAL);
  ssd1306_printFixed(90, 56, connection_failed_char, STYLE_NORMAL);
}

void consumptionScreen(){
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_clearScreen();
  ssd1306_printFixed(0,  8, "L1 consum: ", STYLE_NORMAL);
  ssd1306_printFixed(60, 8, consumptionL1_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  24, "L2 consum: ", STYLE_NORMAL);
  ssd1306_printFixed(60, 24, consumptionL2_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  40, "L3 consum: ", STYLE_NORMAL);
  ssd1306_printFixed(60, 40, consumptionL3_char, STYLE_NORMAL);
  ssd1306_printFixed(0,  56, "Conn failed: ", STYLE_NORMAL);
  ssd1306_printFixed(90, 56, connection_failed_char, STYLE_NORMAL);
}


