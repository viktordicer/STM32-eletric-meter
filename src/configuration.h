//-------------HARDWARE VARIABLES
//ETHERNET
#define RESET_PIN               PA0 // Reset of W5500
#define GREEN_LED               PC13

//Water flow sensors
#define INTERRUPT_L1              PB15 // internal circuit flow sensor
#define INTERRUPT_L2              PB14 // internal circuit flow sensor
#define INTERRUPT_L3              PB13 // internal circuit flow sensor

#define INTERRUPT_L1_OVEN         PA12 // internal circuit flow sensor
#define INTERRUPT_L2_REFRIGERATOR PA11 // internal circuit flow sensor
#define INTERRUPT_L3_DISHWASHER   PA9 // internal circuit flow sensor

//ETHERNET ADDRESS
int IP_ADDRESS[4] =        {192, 168, 0 , 175};
uint8_t MAC[6] =          {0x02, 0x06, 0xA4, 0x01, 0x04, 0x06};

//EEPROM MEMORY ADDRESS
uint8_t EEPROM_ADDRESS[6] =          {0x00, 0x10, 0x20, 0x30, 0x40, 0x50};

//TIME VARIABLES
uint32_t measure_interval  =  120000; // delay in ms
uint32_t eeprom_interval  =  1800000; // delay in ms

//MQTT
#define MQTT_SERVER_IP           "192.168.0.107"   // IP address of MQTT broker
#define MQTT_CLIENT_ID           "Foyer_el_room" // require ID for MQTT client
#define USERNAME                 "viktor"         // username MQTT broker
#define PASSWORD                 "viktor"         // password MQTT broker

//TOPICS
//Main phases
#define HA_TOPIC                     "homeassistant/status"
#define POWER_TOPIC                  "sensor/foyer/power"
#define STATUS_TOPIC                 "sensor/foyer/status"

#define SET_L1_TOTAL_TOPIC           "sensor/foyer/set_L1" //set total consumption after restart
#define SET_L2_TOTAL_TOPIC           "sensor/foyer/set_L2" //set total consumption after restart
#define SET_L3_TOTAL_TOPIC           "sensor/foyer/set_L3" //set total consumption after restart
#define SET_OVEN_TOTAL_TOPIC         "sensor/foyer/set_oven" //set total consumption after restart
#define SET_REFRI_TOTAL_TOPIC        "sensor/foyer/set_refri" //set total consumption after restart
#define SET_DISH_TOTAL_TOPIC         "sensor/foyer/set_dish" //set total consumption after restart
#define SET_INTERVAL_TOPIC           "sensor/foyer/set_interval" //set total consumption after restart

#define L1_POWER_TOPIC               "sensor/foyer/L1_power"
#define L2_POWER_TOPIC               "sensor/foyer/L2_power"
#define L3_POWER_TOPIC               "sensor/foyer/L3_power"

#define L1_TOTAL_TOPIC               "sensor/foyer/L1_total"
#define L2_TOTAL_TOPIC               "sensor/foyer/L2_total"
#define L3_TOTAL_TOPIC               "sensor/foyer/L3_total"

#define L1_OVEN_TOPIC                "sensor/foyer/L1_oven"
#define L2_REFRIGERATOR_TOPIC        "sensor/foyer/L2_refrigerator"
#define L2_DISHWASHER_TOPIC          "sensor/foyer/L3_dishwasher"

#define L1_OVEN_TOTAL_TOPIC          "sensor/foyer/L1_oven_total"
#define L2_REFRIGERATOR_TOTAL_TOPIC  "sensor/foyer/L2_refrigerator_total"
#define L2_DISHWASHER_TOTAL_TOPIC    "sensor/foyer/L3_dishwasher_total"

#define L1_CONSUM_INC_TOPIC          "sensor/foyer/L1_con_inc"
#define L2_CONSUM_INC_TOPIC          "sensor/foyer/L2_con_inc"
#define L3_CONSUM_INC_TOPIC          "sensor/foyer/L3_con_inc"

#define L1_OVEN_INC_TOPIC            "sensor/foyer/L1_oven_inc"
#define L2_REFRIGERATOR_INC_TOPIC    "sensor/foyer/L2_refrigerator_inc"
#define L3_DISHWASHER_INC_TOPIC      "sensor/foyer/L3_dishwasher_inc"

#define FOYER_CONNECTION_FAILED      "sensor/foyer/connection_failed"