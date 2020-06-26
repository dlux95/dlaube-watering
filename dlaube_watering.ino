#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_system.h"
#include <rom/rtc.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "config.h"

typedef enum{
  STATE_NO,
  STATE_IDLE,
  STATE_PUMPING,
  STATE_DRYOUT,
  STATE_ERROR
} State;

typedef struct{
  byte id;
  Adafruit_ADS1115* adc;
  byte adcChannel;
  byte pumpPin;
  State state;
  long stateTimer;
  int16_t lastValue;
  float lastMoisture;
  float sensorMin;
  float sensorMax;
  float thresholdMoist;
  float thresholdDry;
  int pumpCount;
  int startTimer;
  int pumpTime;
} Plant;

Plant plants[NUM_PLANTS];
int isActive = 0;
int debug_timer = 0;

hw_timer_t *timer = NULL;
void IRAM_ATTR resetModule(){
    ESP.restart();
}


char* get_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1 : return "POWERON_RESET";          /**<1, Vbat power on reset*/
    case 3 : return "SW_RESET";               /**<3, Software reset digital core*/
    case 4 : return "OWDT_RESET";             /**<4, Legacy watch dog reset digital core*/
    case 5 : return "DEEPSLEEP_RESET";        /**<5, Deep Sleep reset digital core*/
    case 6 : return "SDIO_RESET";             /**<6, Reset by SLC module, reset digital core*/
    case 7 : return "TG0WDT_SYS_RESET";       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : return "TG1WDT_SYS_RESET";       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : return "RTCWDT_SYS_RESET";       /**<9, RTC Watch dog Reset digital core*/
    case 10 : return "INTRUSION_RESET";       /**<10, Instrusion tested to reset CPU*/
    case 11 : return "TGWDT_CPU_RESET";       /**<11, Time Group reset CPU*/
    case 12 : return "SW_CPU_RESET";          /**<12, Software reset CPU*/
    case 13 : return "RTCWDT_CPU_RESET";      /**<13, RTC Watch dog Reset CPU*/
    case 14 : return "EXT_CPU_RESET";         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : return "RTCWDT_BROWN_OUT_RESET";/**<15, Reset when the vdd voltage is not stable*/
    case 16 : return "RTCWDT_RTC_RESET";      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : return "NO_MEAN";
  }
}

void setup() {
  Serial.begin(115200);

  randomSeed(analogRead(32));

  pinMode(wifi::led, OUTPUT);
  pinMode(mqtt::led, OUTPUT);
  digitalWrite(wifi::led, HIGH);
  digitalWrite(mqtt::led, HIGH);

  for(int i = 0; i < NUM_PLANTS; i++){
    plants[i].id = i;
    if( i < 4 ){
      plants[i].adcChannel = i;
      plants[i].adc = &adc1;
    }else{
      plants[i].adcChannel = i-4;
      plants[i].adc = &adc2;
    }
    plants[i].pumpPin = pins[i];
    plants[i].state = STATE_DRYOUT;
    plants[i].stateTimer = 60 * SECONDS + millis() + random(5, 60) * SECONDS;
    plants[i].sensorMin = 0;
    plants[i].sensorMax = 4095;
    plants[i].thresholdMoist = 100;
    plants[i].thresholdDry = 0;
    plants[i].pumpCount = 0;
    plants[i].startTimer = 60 * SECONDS;
    plants[i].pumpTime = 10;

    pinMode(plants[i].pumpPin, OUTPUT);
    digitalWrite(plants[i].pumpPin, HIGH);
  }
  
  setup_wifi();
  setup_mqtt();

  adc1.begin();
  adc2.begin();

  timer = timerBegin(0, 80, true); //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);
  timerAlarmWrite(timer, 60000000L, false); //set time in us
  timerAlarmEnable(timer); //enable interrupt
}

void setup_wifi() {
  WiFi.begin(wifi::ssid, wifi::password);
  WiFi.setHostname("dlaube-water-00");
}

void setup_mqtt(){
  mqtt::client.setServer(mqtt::server, mqtt::port);
  mqtt::client.setCallback(callback);
}

void callback(char* raw_topic, byte* raw_message, unsigned int len) {
  char cmsg[len+1];
  for (int i = 0; i < len; i++) {
    cmsg[i] = (char)raw_message[i];
    cmsg[i+1] = (char) 0;
  }
  String msg = String(cmsg);
  String topic = String(raw_topic);
  topic.remove(0, String(mqtt::topic).length());

  Serial.println("MQTT: " + topic + "=" + msg);

  int plantNumber = topic.substring(11, 12).toInt();
  String command = topic.substring(13);

  Serial.println(plantNumber);
  Serial.println("CMD: " + command);

  if (command == "sensorMin"){
    int value = msg.toInt();
    plants[plantNumber].sensorMin = value;
  }
  if (command == "sensorMax"){
    int value = msg.toInt();
    plants[plantNumber].sensorMax = value;
  }
  if (command == "thresholdDry"){
    float value = msg.toFloat();
    plants[plantNumber].thresholdDry = value;
  }
  if (command == "thresholdMoist"){
    float value = msg.toFloat();
    plants[plantNumber].thresholdMoist = value;
  }
  if (command == "pumpTime"){
    int value = msg.toInt();
    plants[plantNumber].pumpTime = value;
  }

  if( plantNumber == 8 && command == "active"){
    isActive = msg.toInt();
  }
}

void wifi_reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    wifi::counter++;
    if ( wifi::counter > 50){
      Serial.println("WiFi not connected. Connecting...");
      WiFi.reconnect();
      wifi::counter = 0;
    }
  }
}

void mqtt_reconnect() {
  if (!mqtt::client.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.println("MQTT not connected. Connecting...");
    if (mqtt::client.connect(mqtt::topic, mqtt::user, mqtt::password)) {
      mqtt_subscribe();
    }
  }
}

void mqtt_subscribe(){
  for(int i = 0; i < 8; i++){
   value_subscribe(i, "sensorMin");
   value_subscribe(i, "sensorMax");
   value_subscribe(i, "thresholdDry");
   value_subscribe(i, "thresholdMoist");
   value_subscribe(i, "pumpTime");
  }
  value_subscribe(8, "active");
}

void value_subscribe(int index, String key){
  char strbuffer[128];
  sprintf(strbuffer, "%s/set_state/%d/%s", mqtt::topic, index, key.c_str());
  mqtt::client.subscribe(strbuffer);
}

void value_publish(int index, String key, String value){
  char strbuffer[128];
  sprintf(strbuffer, "%s/state/%d/%s", mqtt::topic, index, key.c_str());
  mqtt::client.publish(strbuffer, value.c_str(), true);
}

char* state_to_string(int state){
  switch(state){
    case STATE_NO:
      return "INACTIVE";
    case STATE_IDLE:
      return "IDLE";
    case STATE_PUMPING:
      return "WATER";
    case STATE_DRYOUT:
      return "DRYOUT";
    case STATE_ERROR:
      return "ERROR";
    default:
      return "INVALID";
    }
}

void state_publish(Plant* plant){
  char tmpBuffer[128];
  value_publish(plant->id, "state", state_to_string(plant->state));
  value_publish(plant->id, "moisture", dtostrf(plant->lastMoisture, 0, 2, tmpBuffer));
  value_publish(plant->id, "sensorValue", dtostrf(plant->lastValue, 0, 2, tmpBuffer));
  value_publish(plant->id, "sensorMin", dtostrf(plant->sensorMin, 0, 2, tmpBuffer));
  value_publish(plant->id, "sensorMax", dtostrf(plant->sensorMax, 0, 2, tmpBuffer));
  value_publish(plant->id, "thresholdMoist", dtostrf(plant->thresholdMoist, 0, 2, tmpBuffer));
  value_publish(plant->id, "thresholdDry", dtostrf(plant->thresholdDry, 0, 2, tmpBuffer));
}

void state_switch(Plant* plant, State state, long stateTime){
  Serial.printf("Plant %d changed from %s to %s\n", plant->id, state_to_string(plant->state), state_to_string(state));
  plant->state = state;
  plant->stateTimer = stateTime;

  if(plant->startTimer < 0){
    state_publish(plant);
  }
}

bool state_is_elapsed(Plant* plant){
  return plant->stateTimer <= 0;
}

void state_machine(Plant* plant){
  if (plant->state == STATE_NO){
    return;
  }

  plant->lastValue = plant->adc->readADC_SingleEnded(plant->adcChannel);
  float new_moisture = map_float(plant->lastValue, plant->sensorMin, plant->sensorMax, 0, 100);
  new_moisture = constrain(new_moisture, 0, 100);
  plant->lastMoisture = new_moisture * 0.01f + plant->lastMoisture * 0.99f;

  plant->stateTimer -= STATE_LOOP_MILLIS;
  if(plant->startTimer >= 0){
    plant->startTimer -= STATE_LOOP_MILLIS;
  }else{
    switch(plant->state){
      case STATE_IDLE:
        if(plant->pumpPin != -1)
          digitalWrite(plant->pumpPin, HIGH);
        if (state_is_elapsed(plant)){
          if (plant->lastMoisture < plant->thresholdMoist && isActive){
            state_switch(plant, STATE_PUMPING, plant->pumpTime*SECONDS);
          }else{
            plant->pumpCount = 0;
            state_switch(plant, STATE_DRYOUT, 60*SECONDS);
          }
        }
        break;
      case STATE_PUMPING:
        if(plant->pumpPin != -1)
          digitalWrite(plant->pumpPin, LOW);
        if (state_is_elapsed(plant)){
          plant->pumpCount++;
          if(plant->pumpCount > 24){
            state_switch(plant, STATE_ERROR, 60 * SECONDS);
          }else{
            state_switch(plant, STATE_IDLE, 60 * SECONDS);
          }
        }
        break;
      case STATE_DRYOUT:
        if(plant->pumpPin != -1)
          digitalWrite(plant->pumpPin, HIGH);
        if (state_is_elapsed(plant)){
          if (plant->lastMoisture > plant->thresholdDry || !isActive){
            state_switch(plant, STATE_DRYOUT, 60*SECONDS);
          }else{
            state_switch(plant, STATE_PUMPING, plant->pumpTime*SECONDS);
          }
        }
        break;
      case STATE_ERROR:
        if(plant->pumpPin != -1)
          digitalWrite(plant->pumpPin, HIGH);
        if (state_is_elapsed(plant)){
          state_switch(plant, STATE_ERROR, 60*SECONDS);
        }
        break;
    }
  }
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  wifi_reconnect();
  mqtt_reconnect();
  
  mqtt::client.loop();

  debug_timer -= STATE_LOOP_MILLIS;
  if(debug_timer <= 0){
    debug_timer = DEBUG_MILLIS;

    char tmpBuffer[128];
    value_publish(8, "heap_free", itoa(ESP.getFreeHeap(), tmpBuffer, 10));
    value_publish(8, "reset_reason_0", get_reset_reason(rtc_get_reset_reason(0)));
    value_publish(8, "reset_reason_1", get_reset_reason(rtc_get_reset_reason(1)));
    Serial.print("Free Heap: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Active: ");
    Serial.println(isActive);
    Serial.print("States:\t");
    for (int i = 0; i < NUM_PLANTS; i++){
      Serial.printf("%8s(%02d)\t" , state_to_string(plants[i].state), plants[i].stateTimer/SECONDS);
    }
    Serial.println();
    Serial.print("Sensor:\t");
    for (int i = 0; i < NUM_PLANTS; i++){
      char strbuffer[100];
      Serial.printf("%11d\t", (int) plants[i].lastValue);
    }
    Serial.println();
  }
  for (int i = 0; i < NUM_PLANTS; i++){
    state_machine(&plants[i]);
  }
  delay(STATE_LOOP_MILLIS);
  if(WiFi.status() == WL_CONNECTED){
    digitalWrite(wifi::led, !(wifi::led_counter > 250) );
    wifi::led_counter = (wifi::led_counter + STATE_LOOP_MILLIS) % 500;
  }else{
    digitalWrite(wifi::led, HIGH);
  }
  if(mqtt::client.connected()){
    digitalWrite(mqtt::led, !(mqtt::led_counter > 250) );
    mqtt::led_counter = (mqtt::led_counter + STATE_LOOP_MILLIS) % 500;
  }else{
    digitalWrite(mqtt::led, HIGH);
  }
  timerWrite(timer, 0);
}
