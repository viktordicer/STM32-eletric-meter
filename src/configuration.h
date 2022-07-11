//-------------HARDWARE VARIABLES
//ETHERNET
#define RESET_PIN               PA0 // Reset of W5500

//Water flow sensors
#define INTERRUPT_L1            PB15 // internal circuit flow sensor
#define INTERRUPT_L2            PB14 // internal circuit flow sensor
#define INTERRUPT_L3            PB13 // internal circuit flow sensor

//ETHERNET ADDRESS
int IP_ADDRESS[4] =        {192, 168, 0 , 175};
uint8_t MAC[6] =          {0x02, 0x06, 0xA4, 0x01, 0x04, 0x06};

//MQTT
#define MQTT_SERVER_IP           "192.168.0.107"   // IP address of MQTT broker
#define MQTT_CLIENT_ID           "Foyer_el_room" // require ID for MQTT client
#define USERNAME                 "viktor"         // username MQTT broker
#define PASSWORD                 "viktor"         // password MQTT broker

//TOPICS
#define L1_POWER_TOPIC              "sensor/foyer/L1_power"
#define L2_POWER_TOPIC              "sensor/foyer/L2_power"
#define L3_POWER_TOPIC              "sensor/foyer/L3_power"

#define L1_CONSUM_INC_TOPIC          "sensor/foyer/L1_con_inc"
#define L2_CONSUM_INC_TOPIC          "sensor/foyer/L2_con_inc"
#define L3_CONSUM_INC_TOPIC          "sensor/foyer/L3_con_inc"
#define FOYER_CONNECTION_FAILED      "sensor/foyer/connection_failed"

//VARIABLES
#define MAX_PHASE_COUNTS   20000 //reset phase count after reach max

// TIME VARIABLES
#define MQTT_SENDING_DEALY            20 // mqtt delay between publish
#define MEASURE_INTERVAL              360000 // delay in ms
