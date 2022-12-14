#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include <PZEM004Tv30.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN) && !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

#endif

#if defined(USE_SOFTWARE_SERIAL)
//SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
//PZEM004Tv30 pzem(pzemSWSerial);

#elif defined(ESP32)
PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);
#else
PZEM004Tv30 pzem(PZEM_SERIAL);

#endif

float pzem_voltage, pzem_current, pzem_power, pzem_energy, pzem_frequency, pzem_pf; 


#define WIFI_AP "bukanwifi"
#define WIFI_PASSWORD "ahmadhikam"

#define TOKEN "BRlxThdnrpVh3Hqz7Dny"

#define GPIO23 2 //relay Pin 
//#define GPIO2 2

#define GPIO23_PIN 14
//#define GPIO2_PIN 5

char thingsboardServer[] = "thingsboard.cloud";

WiFiClient wifiClient;

PubSubClient client(wifiClient);

int status = WL_IDLE_STATUS;

ThingsBoard tb(wifiClient);

unsigned long lastSend;

// We assume that all GPIOs are LOW
boolean gpioState[] = {false, false};

LiquidCrystal_I2C lcd(0x27,16,2);
#define DHTPIN            4         // Pin which is connected to the DHT sensor.

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
float Temp = 0;
float Humid = 0;
 float DO = 18;;

void SensorDHT(){
  
  // Delay between measurements.
  
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
    Temp = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Error reading temperature!");
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
    Humid = event.relative_humidity;
 
    
  }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Temp : ");
    lcd.setCursor(8,0);
    lcd.print(Temp);
    lcd.setCursor(14,0);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print("Humidity: ");
    lcd.setCursor(10,1);
    lcd.print(event.relative_humidity);
}

void pzemRead(){

  /**
   * Fungsi untuk menerima data dari sensor PZEM 004T
   * Disini tedapat beberapa variabel yang masing-masing menympan nilai yang berbeda
    */
   
  pzem_voltage = pzem.voltage();
  pzem_current = pzem.current();
  pzem_power = pzem.power();
  pzem_energy = pzem.energy();
  pzem_frequency = pzem.frequency();
  pzem_pf = pzem.pf();
}

void pzemMonitor(){

  /**
   * Fungsi untuk mendebug(menampilakan pada serial monitor) nilai-nilai yang didapatkan dari vaiabel yang diolah pada fungsi pzemRead.
   */
   
  pzemRead();
  
  if(isnan(pzem_voltage)){
        Serial.println("Error reading voltage");
    } else if (isnan(pzem_current)) {
        Serial.println("Error reading current");
    } else if (isnan(pzem_power)) {
        Serial.println("Error reading power");
    } else if (isnan(pzem_energy)) {
        Serial.println("Error reading energy");
    } else if (isnan(pzem_frequency)) {
        Serial.println("Error reading frequency");
    } else if (isnan(pzem_pf)) {
        Serial.println("Error reading power factor");
    } else {

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(pzem_voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(pzem_current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(pzem_power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(pzem_energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(pzem_frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pzem_pf);
    }

    Serial.println();
    delay(2000);
}

void powerSystem(int relayMode){

  /**
   * Fungsi untuk menghidupkan dan mematikan relay 
   * Digunakan untuk nyala mati aerator 
   * Panggil fungsi dengan parameter 0 untuk menyalakan relay dan 1 untuk mematikan relay
   */
   
  if(relayMode == 0){
    // Turn the relay switch ON 
    digitalWrite(GPIO23, LOW);// set relay pin to low 
    Serial.println("Relay ON ");
  }else if(relayMode == 1){
    // Turn the relay switch OFF 
    digitalWrite(GPIO23, HIGH);// set relay pin to HIGH
    Serial.println("Relay OFF ");
  }

}

void PID(){
  DO = 25;
  while(DO >= 7){
    DO = DO - 0.5;
    //delay(1500);
    if (DO == 7){
      DO = random(6,8);
    }
  }
}
void getAndSendData(){
   Serial.println("Collecting temperature data.");
   
  //PID();
  
  float error = random(0, 2.2);
    //delay(1500);
    if (DO >= 7){
      DO = DO - error;
    }else{
      DO = random(6,9);
    }
  float salinity = random(1,100);
  float temperature = random(1,100);
  float pH = random(1,100);
 
  

  // Check if any reads failed and exit early (to try again).
  if (isnan(salinity) || isnan(temperature) || isnan(pH) || isnan(DO)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.println("Sending data to ThingsBoard:");
  Serial.print("Salinity: ");
  Serial.print(salinity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(Temp);
  Serial.println(" *C ");
  Serial.print("pH: ");
  Serial.print(pH);
  Serial.print(" %\t");
  Serial.print("DO: ");
  Serial.print(DO);
  Serial.print("humidity: ");
  Serial.print(Humid);

  //tb.sendTelemetryFloat("temperature", temperature);
  tb.sendTelemetryFloat("temperature", Temp);
  tb.sendTelemetryFloat("humidity", Humid);
  tb.sendTelemetryFloat("salinity", salinity);
  tb.sendTelemetryFloat("DO", DO);
  tb.sendTelemetryFloat("pH", pH);
  tb.sendTelemetryFloat("Power", pzem_power);
  tb.sendTelemetryFloat("Energy", pzem_energy);
  tb.sendTelemetryFloat("Power_Faktor", pzem_pf);

}

void setup() {
  Serial.begin(115200);
  // Set output mode for all GPIO pins
  pinMode(GPIO23, OUTPUT);
//  pinMode(GPIO2, OUTPUT);
  delay(10);
  dht.begin();
  lcd.init();
  lcd.backlight();
  sensor_t sensor;
  dht.humidity().getSensor(&sensor);
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);

  lastSend = 0;
}

void loop() {
  SensorDHT();
  if ( !client.connected()  ) {
    reconnect();
  }

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    getAndSendData();
//    tb.loop();
    lastSend = millis();
  }

  
  client.loop();

  pzemMonitor();
}

// The callback for when a PUBLISH message is received from the server.
void on_message(const char* topic, byte* payload, unsigned int length) {

  Serial.println("On message");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(json);

  // Decode JSON request
  StaticJsonDocument<200> jsonBuffer;
  DeserializationError data = deserializeJson(jsonBuffer, (char*)json);

  if (data)
  {
    Serial.println("parseObject() failed");
    return;
  }

  // Check request method
  String methodName = String((const char*)jsonBuffer["method"]);

  if (methodName.equals("getGpioStatus")) {
    // Reply with GPIO status
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  } else if (methodName.equals("setGpioStatus")) {
    // Update GPIO status and reply
    set_gpio_status(jsonBuffer["params"]["pin"], jsonBuffer["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  }
}

String get_gpio_status() {
  // Prepare gpios JSON payload string
  StaticJsonDocument<200> jsonBuffer;
  JsonObject data = jsonBuffer.to<JsonObject>();
  data[String(GPIO23_PIN)] = gpioState[0] ? true : false;
//  data[String(GPIO2_PIN)] = gpioState[1] ? true : false;
  char payload[256];
  serializeJson(jsonBuffer, payload);
  String strPayload = String(payload);
  Serial.print("Get gpio status: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
  if (pin == GPIO23_PIN) {
    // Output GPIOs state
    digitalWrite(GPIO23, enabled ? HIGH : LOW);
    // Update GPIOs state
    gpioState[0] = enabled;
  } 
//  else if (pin == GPIO2_PIN) {
//    // Output GPIOs state
//    digitalWrite(GPIO2, enabled ? HIGH : LOW);
//    // Update GPIOs state
//    gpioState[1] = enabled;
//  }
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }

  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED]" );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
