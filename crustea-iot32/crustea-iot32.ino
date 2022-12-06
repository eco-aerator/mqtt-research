// Kode untuk mendefine sensor PZEM 004T 
#include <PZEM004Tv30.h>


#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN) && !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

#endif


/* Hardware Serial2 is only available on certain boards.
 * For example the Arduino MEGA 2560
*/
#if defined(USE_SOFTWARE_SERIAL)
//#include <SoftwareSerial.h>
/*************************
 *  Use SoftwareSerial for communication
 * ---------------------
 * 
 * The ESP32 platform does not support the SoftwareSerial as of now 
 * Here we initialize the PZEM on SoftwareSerial with RX/TX pins PZEM_RX_PIN and PZEM_TX_PIN
 */
//SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
//PZEM004Tv30 pzem(pzemSWSerial);

#elif defined(ESP32)
/*************************
 *  ESP32 initialization
 * ---------------------
 * 
 * The ESP32 HW Serial interface can be routed to any GPIO pin 
 * Here we initialize the PZEM on PZEM_SERIAL with RX/TX pins PZEM_RX_PIN and PZEM_TX_PIN
 */
PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

#else
/*************************
 *  Arduino/ESP8266 initialization
 * ---------------------
 * 
 * Not all Arduino boards come with multiple HW Serial ports.
 * Serial2 is for example available on the Arduino MEGA 2560 but not Arduino Uno!
 * The ESP32 HW Serial interface can be routed to any GPIO pin 
 * Here we initialize the PZEM on PZEM_SERIAL with default pins
 */
PZEM004Tv30 pzem(PZEM_SERIAL);

#endif

float pzem_voltage, pzem_current, pzem_power, pzem_energy, pzem_frequency, pzem_pf; 

// Kode untuk relay 
int relayPin = 23; //set pin 8 for relay output

// Kode untuk mendefine ESP32 dengan WiFi dan MQTT API
#include <WiFi.h>
#include <PubSubClient.h>

#define WIFI_SSID "ipongggggg"
#define WIFI_PASSWORD "12345678"

// thingsboard

#include <ThingsBoard.h>
#define TOKEN "EBII_ACCESS_TOKEN"
#define thingsboardServer "thingsboard.cloud"
WiFiClient wifiClient;

ThingsBoard tb(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

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
    digitalWrite(relayPin, LOW);// set relay pin to low 
    Serial.println("Relay ON ");
  }else if(relayMode == 1){
    // Turn the relay switch OFF 
    digitalWrite(relayPin, HIGH);// set relay pin to HIGH
    Serial.println("Relay OFF ");
  }

}

void getAndSendData(){
   Serial.println("Collecting temperature data.");
   
  float salinity = random(1,100);
  float temperature = random(1,100);
  float pH = random(1,100);
  float DO = random(1,100);

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
  Serial.print(temperature);
  Serial.println(" *C ");
  Serial.print("pH: ");
  Serial.print(pH);
  Serial.print(" %\t");
  Serial.print("DO: ");
  Serial.print(DO);

  tb.sendTelemetryFloat("temperature", temperature);
  tb.sendTelemetryFloat("salinity", salinity);
  tb.sendTelemetryFloat("DO", DO);
  tb.sendTelemetryFloat("pH", pH);

}

void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}

void setup() {
  Serial.begin(9600);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  lastSend = 0;
  
  pinMode(relayPin, OUTPUT);

}

void loop() {

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    getAndSendData();
    lastSend = millis();
  }

  tb.loop();
  
  // memantau sensor PZEM
  pzemMonitor();

  
  

}
