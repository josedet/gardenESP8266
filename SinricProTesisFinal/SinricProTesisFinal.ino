/* 
  * ADVANCED example for: how to use up to N SinricPro Switch devices on one ESP module  
  *                       to control N relays and N flipSwitches for manually control: 
  * - setup N SinricPro switch devices 
  * - setup N relays 
  * - setup N flipSwitches to control relays manually 
  *   (flipSwitch can be a tactile button or a toggle switch and is setup in line #52) 
  *  
  * - handle request using just one callback to switch relay 
  * - handle flipSwitches to switch relays manually and send event to sinricPro server 
  *  
  * - SinricPro deviceId and PIN configuration for relays and buttins is done in std::map<String, deviceConfig_t> devices 
  *  
  * If you encounter any issues: 
  * - check the readme.md at https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md 
  * - ensure all dependent libraries are installed 
  *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#arduinoide 
  *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#dependencies 
  * - open serial monitor and check whats happening 
  * - check full user documentation at https://sinricpro.github.io/esp8266-esp32-sdk 
  * - visit https://github.com/sinricpro/esp8266-esp32-sdk/issues and check for existing issues or open a new one 
  * 
  * 
  * This program has been modified to be used for garden watering. 
  */ 
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "Wire.h"
#include "Adafruit_GFX.h"
#include "OakOLED.h"
#include <Adafruit_SSD1306.h>

///////////////////
#define ANCHO_PANTALLA 128 // Ancho de la pantalla OLED
#define ALTO_PANTALLA 64 // Alto de la pantalla OLED

#define OLED_RESET     -1 // Pin reset incluido en algunos modelos de pantallas (-1 si no disponemos de pulsador). 
#define DIRECCION_PANTALLA 0x3C //Dirección de comunicacion: 0x3D para 128x64, 0x3C para 128x32

Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, OLED_RESET);

#define LOGO_WIDTH    84
#define LOGO_HEIGHT   52
#define SENSOR  15

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned long flowMilliLitres;
unsigned int totalMilliLitres;
float flowLitres;
float totalLitres;

int porcentaje=0;
 
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

const unsigned char PROGMEM logo[] = {
0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 
0x10, 0x80, 0x00, 0x00, 0x03, 0xff, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x1f, 
0xff, 0xf1, 0xf0, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xe3, 0xf8, 0x00, 0x00, 
0x00, 0x10, 0x80, 0x00, 0x00, 0xff, 0xff, 0xe3, 0xf8, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x03, 
0xff, 0x00, 0xe7, 0xfc, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x07, 0xf8, 0x00, 0x07, 0xfc, 0x00, 
0x00, 0x00, 0x10, 0x80, 0x00, 0x0f, 0xe0, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 
0x1f, 0xc0, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x3f, 0x00, 0x00, 0x03, 0xfc, 
0x06, 0x00, 0x00, 0x10, 0x80, 0x00, 0x3e, 0x00, 0x00, 0x03, 0xf8, 0x0f, 0x00, 0x00, 0x10, 0x80, 
0x00, 0x7e, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x80, 0x00, 0x10, 0x80, 0x00, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0x80, 0x00, 0x10, 0x80, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x10, 
0x80, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe0, 0x00, 0x10, 0x80, 0x01, 0xf0, 0x00, 0x00, 
0x00, 0x00, 0x7f, 0xe0, 0x00, 0x10, 0x80, 0x01, 0xf0, 0xe0, 0x70, 0x00, 0x00, 0x7f, 0xe0, 0x00, 
0x10, 0x80, 0x01, 0xe1, 0xf0, 0xf0, 0x00, 0xc0, 0x7f, 0xe0, 0x00, 0x10, 0x80, 0x03, 0xe3, 0xb9, 
0xe0, 0x00, 0xc0, 0x7f, 0xe0, 0x00, 0x10, 0x80, 0x03, 0xe3, 0xb9, 0xc0, 0x01, 0xe0, 0x7f, 0xe0, 
0x00, 0x10, 0x80, 0x03, 0xc1, 0xfb, 0x80, 0x03, 0xf0, 0x7f, 0xe0, 0x00, 0x10, 0x80, 0x03, 0xc0, 
0xe7, 0x80, 0x03, 0xf0, 0x3f, 0xc0, 0x00, 0x10, 0x80, 0x03, 0xc0, 0x0f, 0x00, 0x07, 0xf8, 0x0f, 
0x00, 0x00, 0x10, 0x80, 0x03, 0xc0, 0x1e, 0x70, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x10, 0x80, 0x03, 
0xc0, 0x1e, 0xf8, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x10, 0x80, 0x03, 0xc0, 0x3d, 0xdc, 0x1f, 0xfe, 
0x00, 0x00, 0x00, 0x10, 0x80, 0x03, 0xc0, 0x79, 0xdc, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x10, 0x80, 
0x03, 0xe0, 0xf0, 0xf8, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x10, 0x80, 0x03, 0xe0, 0xe0, 0x78, 0x7f, 
0xff, 0x80, 0x00, 0x00, 0x10, 0x80, 0x01, 0xe0, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x10, 
0x80, 0x01, 0xf0, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x10, 0x80, 0x01, 0xf0, 0x00, 0x00, 
0xff, 0xff, 0xc0, 0x00, 0x00, 0x10, 0x80, 0x00, 0xf8, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 
0x10, 0x80, 0x00, 0xf8, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x10, 0x80, 0x00, 0x7c, 0x00, 
0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x10, 0x80, 0x00, 0x7e, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x10, 0x80, 0x00, 0x3f, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x10, 0x80, 0x00, 0x1f, 
0x80, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x10, 0x80, 0x00, 0x0f, 0xe0, 0x00, 0x3f, 0xff, 0x00, 
0x00, 0x00, 0x10, 0x80, 0x00, 0x07, 0xf0, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 
0x03, 0xfe, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x01, 0xff, 0xff, 0x83, 0xf0, 
0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 
0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x07, 0xff, 0xf0, 
0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 
0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10
};



///////////////////////////
 // Uncomment the following line to enable serial debug output 
 //#define ENABLE_DEBUG 
  
 #ifdef ENABLE_DEBUG 
        #define DEBUG_ESP_PORT Serial 
        #define NODEBUG_WEBSOCKETS 
        #define NDEBUG 
 #endif  
  
 #include <Arduino.h> 
 #ifdef ESP8266  
        #include <ESP8266WiFi.h> 
 #endif  
 #ifdef ESP32    
        #include <WiFi.h> 
 #endif 
  
 #include "SinricPro.h" 
 #include "SinricProSwitch.h" 
  
 #include <map> 
  
 #define WIFI_SSID         "COLOCAR EL SSID DE LA RED INALAMBRICA"     
 #define WIFI_PASS         "COLOCAR LA CONTRASEÑA DE LA RED" 
 #define APP_KEY           "COLOCAR LA LLAVE DE SINRIC PRO"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx" 
 #define APP_SECRET        "COLOCAR LA CONTRASEÑA DE SINRIC PRO"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx" 
 
 #define BAUD_RATE   9600 
  
 #define DEBOUNCE_TIME 250 
  
 typedef struct {      // struct for the std::map below 
   int relayPIN; 
   int flipSwitchPIN; 
 } deviceConfig_t; 

OakOLED oled;
  const long utcOffsetInSeconds = -18000;
    // Define NTP Client to get time
  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

  WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

 // this is the main configuration 
 // please put in your deviceId, the PIN for Relay and PIN for flipSwitch 
 // this can be up to N devices...depending on how much pin's available on your device ;) 
 // right now we have 4 devicesIds going to 4 relays and 4 flip switches to switch the relay manually 
 std::map<String, deviceConfig_t> devices = { 
     {"COLOCARD ID generado", {  D3, D8 }}, 
   
 }; 
  
 typedef struct {      // struct for the std::map below 
   String deviceId; 
   bool lastFlipSwitchState; 
   unsigned long lastFlipSwitchChange; 
 } flipSwitchConfig_t; 
  
 std::map<int, flipSwitchConfig_t> flipSwitches;    // this map is used to map flipSwitch PINs to deviceId and handling debounce and last flipSwitch state checks 
                                                   // it will be setup in "setupFlipSwitches" function, using informations from devices map 
  
 void setupRelays() {  
   for (auto &device : devices) {           // for each device (relay, flipSwitch combination) 
     int relayPIN = device.second.relayPIN; // get the relay pin 
     pinMode(relayPIN, OUTPUT);             // set relay pin to OUTPUT 
   } 
 } 
  
 void setupFlipSwitches() { 
   for (auto &device : devices)  {                     // for each device (relay / flipSwitch combination) 
     flipSwitchConfig_t flipSwitchConfig;              // create a new flipSwitch configuration 
  
     flipSwitchConfig.deviceId = device.first;         // set the deviceId 
     flipSwitchConfig.lastFlipSwitchChange = 0;        // set debounce time 
     flipSwitchConfig.lastFlipSwitchState = true;     // set lastFlipSwitchState to false (LOW) 
  
     int flipSwitchPIN = device.second.flipSwitchPIN;  // get the flipSwitchPIN 
  
     flipSwitches[flipSwitchPIN] = flipSwitchConfig;   // save the flipSwitch config to flipSwitches map 
     pinMode(flipSwitchPIN, INPUT);                   // set the flipSwitch pin to INPUT 
   } 
 } 
  
 bool onPowerState(String deviceId, bool &state) 
 { 
   Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off"); 
   int relayPIN = devices[deviceId].relayPIN; // get the relay pin for corresponding device 
   digitalWrite(relayPIN, state);             // set the new relay state 
   return true; 
 } 
  
 void handleFlipSwitches() { 
   unsigned long actualMillis = millis();                                          // get actual millis 
   for (auto &flipSwitch : flipSwitches) {                                         // for each flipSwitch in flipSwitches map 
     unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // get the timestamp when flipSwitch was pressed last time (used to debounce / limit events) 
  
     if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {                    // if time is > debounce time... 
  
       int flipSwitchPIN = flipSwitch.first;                                       // get the flipSwitch pin from configuration 
       bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;           // get the lastFlipSwitchState 
       bool flipSwitchState = digitalRead(flipSwitchPIN);                          // read the current flipSwitch state 
       if (flipSwitchState != lastFlipSwitchState) {                               // if the flipSwitchState has changed... 
 #ifdef TACTILE_BUTTON 
         if (flipSwitchState) {                                                    // if the tactile button is pressed  
 #endif       
           flipSwitch.second.lastFlipSwitchChange = actualMillis;                  // update lastFlipSwitchChange time 
           String deviceId = flipSwitch.second.deviceId;                           // get the deviceId from config 
           int relayPIN = devices[deviceId].relayPIN;                              // get the relayPIN from config 
           bool newRelayState = digitalRead(relayPIN);                            // set the new relay State 
           digitalWrite(relayPIN, newRelayState);                                  // set the trelay to the new state 
  
           SinricProSwitch &mySwitch = SinricPro[deviceId];                        // get Switch device from SinricPro 
           mySwitch.sendPowerStateEvent(!newRelayState);                            // send the event 
 #ifdef TACTILE_BUTTON 
         } 
 #endif       
         flipSwitch.second.lastFlipSwitchState = flipSwitchState;                  // update lastFlipSwitchState 
       } 
     } 
   } 
 } 
  
 void setupWiFi() 
 { 
   Serial.printf("\r\n[Wifi]: Connecting"); 
   WiFi.begin(WIFI_SSID, WIFI_PASS); 
  
   while (WiFi.status() != WL_CONNECTED) 
   { 
     Serial.printf("."); 
     delay(250); 
   } 
   digitalWrite(LED_BUILTIN, HIGH); 
   Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str()); 
 } 
  
 void setupSinricPro() 
 { 
   for (auto &device : devices) 
   { 
     const char *deviceId = device.first.c_str(); 
     SinricProSwitch &mySwitch = SinricPro[deviceId]; 
     mySwitch.onPowerState(onPowerState); 
   } 
  
   SinricPro.begin(APP_KEY, APP_SECRET); 
   SinricPro.restoreDeviceStates(true); 
 } 
  int SensorH =A0;
 void setup() 
 {
   
   Serial.begin(BAUD_RATE); 
   if(!display.begin(SSD1306_SWITCHCAPVCC, DIRECCION_PANTALLA)) {
    Serial.println(F("Fallo en la asignacion de SSD1306"));
  }
  pinMode(SensorH, INPUT);
  pinMode(SENSOR, INPUT_PULLUP); //sensor d8

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("SISTEMA DE RIEGO UNSA");
  display.display();
  display.drawLine(0, 10, 128, 10, WHITE);
  display.display();

  display.drawBitmap( (display.width() - LOGO_WIDTH ) / 2,((display.height()- LOGO_HEIGHT) / 2 )+7, logo, LOGO_WIDTH, LOGO_HEIGHT, WHITE);
  display.display();
  delay(2000);
    
   setupRelays(); 
   setupFlipSwitches(); 
   setupWiFi(); 
   setupSinricPro(); 
   oled.begin();
   oled.setTextSize(1);
   oled.setTextColor(1);
   oled.setCursor(0, 0);
   timeClient.begin();

 // Mostrar la información en una pantalla OLED
  oled.setCursor(0, 0);
  Serial.print(daysOfTheWeek[timeClient.getDay()]);
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.print(timeClient.getMinutes());
  Serial.print(":");
  Serial.println(timeClient.getSeconds());
   
 } 

 void flujometro()
 {
   currentMillis = millis();
  if (currentMillis - previousMillis > interval) 
  {
    pulse1Sec = pulseCount;
    pulseCount = 0;
 
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
 
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    flowLitres = (flowRate / 60);
 
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    totalLitres += flowLitres;
  }
 }
 void loop() 
 { 
    SinricPro.handle(); 
    handleFlipSwitches(); 
    int almacenador = analogRead(SensorH);
    porcentaje = map (almacenador, 0, 1024, 100,0);
      
    timeClient.update();
    oled.clearDisplay();
    //Mostrar información de la humedad de suelo, flujometro
    oled.setCursor(0,0);
    oled.println("SISTEMA DE RIEGO UNSA");
    oled.setCursor(0,15);
    oled.print("Humedad Suelo: ");
    oled.print(porcentaje);
    oled.println("%");
    delay(100);
    flujometro();
    oled.setCursor(0,30);  //oled display
    oled.setTextSize(1);
    oled.print("R:");
    oled.print(float(flowRate));
    oled.print(" ");
    oled.print("L/M  ");
    oled.print("V:");
    oled.print(totalLitres);
    oled.print(" ");
    oled.print("L");
    // mostrar la fecga actual
    oled.setCursor(0,45);
    oled.print(daysOfTheWeek[timeClient.getDay()]);
    oled.print("  ");
    oled.print(timeClient.getHours());
    oled.print(":");
    oled.print(timeClient.getMinutes());
    oled.print(":");
    oled.println((timeClient.getSeconds()));
    //Serial.println(timeClient.getFormattedTime());
    oled.display();


 } 
