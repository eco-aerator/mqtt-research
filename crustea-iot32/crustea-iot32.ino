// Kode untuk mendefine sensor PZEM 004T 
#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>

#if defined(ESP32)
    #error "Software Serial is not supported on the ESP32"
#endif

#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#endif

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

float pzem_voltage, pzem_current, pzem_power, pzem_energy, pzem_frequency, pzem_pf; 

// Kode untuk relay 
int relayPin = 8; //set pin 8 for relay output

// Kode untuk mendefine ESP32 dengan WiFi dan MQTT API
#include <WiFi.h>
#include <PubSubClient.h>

#define WIFI_SSID "ipongggggg"
#define WIFI_PASSWORD "12345678"

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
  
  pinMode(relayPin, OUTPUT);

}

void loop() {
  
  // memantau sensor PZEM
  pzemMonitor();

//  if(dataSensor == sekian){
//    // relay nyala, aerator menyala
//    powerSystem(0);
//  }else{
//    // relay mati, aerator mati
//    powerSystem(1);
//  }
  
  

  


  

}
