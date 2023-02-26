#define EEPROM_SIZE 100             /// EEPROM size, non volatile memory for storing averages and statistics when power is cut
#define _mainLoopDelay 50           /// delay for non precise timing, to limit Display and EEPROM writes
#define CAP_SENS_PIN 13             /// Capacitive sensor pin
#define LED_BLUE 32                 /// blue LED pin
#define LED_GREEN 33                /// green LED pin
#define CALIB_SWITCH_PIN 19         /// switch for going into calibration mode

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
//#include <SPIFFS.h>
#include <EEPROM.h>

#include <DNSServer.h>          /// needed for captive AP
#include <AsyncTCP.h>


#include "index.h"              /// file containing html for web interface
#include "lutFreqTransform.h"
#include "max7219.h"            /// containing methods for displaying stuff on 8x 7seg display
#include "liquidLevel.h"        /// containing liquid level class for processing cap. sensor data

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

const char* hotspotSSID = "Water level sensor";   /// ESP hotspot name
const char* hotspotPasswd= "123456789";           /// ESP hotspot passwd

DNSServer dnsServer;                              /// create DNS server
AsyncWebServer server(80);                        /// create webserver on port 80 - AsyncWebServer

Display8x8 displays;

class CaptiveRequestHandler : public AsyncWebHandler {  /// Captive access point class
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    Serial.print("request for: ");
    Serial.print(request->host().c_str());
    Serial.print(" ");
    Serial.println(request->url().c_str());

    request->send(200, "text/html", String(indexStr).c_str());
  }
};


volatile double levelSensPulses = 0;          /// pulses measured from capacitive sensor
volatile double rawPulse = 0;
volatile double newReading = -1;

/// TIMER ISR SETUP
hw_timer_t * timer = NULL;

LiquidLevel levelSens(2000, 0, 16500, 46000); /// new instance of liquid level class for capacitive sensor


/*String getCurrentLevel() {
  return String(rawPulse);
}
String getInfo(){
  /// TankLevel/ TankLiters/ Flow/ TMUntilEmpty/ SpentWater/ AverageFlow/ DischStarted/ Last fill time/ 
 // String webDispStr = String(levelSens.getPercent()) + "/" + String( levelSens.getLiters()) + "/20/5/120000/20/1222/55644/2234";
  return levelSens.getInfoStr();
}*/

void IRAM_ATTR isr() {        /// interrupt service routine, executerd on every sensor pin falling edge
  levelSensPulses++;          /// increase pulse count
}
void IRAM_ATTR timerISR() {   /// timer interrupt routine - executed every 100ms
  newReading = rawPulse = levelSensPulses;  /// TODO: ? this is probably not needed
  levelSens.setFreq((levelSensPulses*10), 100); /// pass number of pulses in the last 100ms to function
  levelSensPulses = 0;
}

void setup(){
  Serial.begin(115200);           /// start DBG serial
  EEPROM.begin(EEPROM_SIZE);      /// start EEprom
  levelSens.loadEEPROM();         /// load max water out, average flow from uController EEPROM
  pinMode(LED_BLUE, OUTPUT);      /// Set LED pins 
  pinMode(LED_GREEN, OUTPUT);
  pinMode(CALIB_SWITCH_PIN, INPUT_PULLUP);      /// switch pin
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  Serial.print("Setting AP (Access Point)");
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  Serial.println(".");

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  WiFi.softAP(hotspotSSID, hotspotPasswd);    /// Start ESP Wifi hotspot
  dnsServer.start(53, "*", WiFi.softAPIP());  /// Start DNS server


  server.on("/currentLevel", HTTP_GET, [](AsyncWebServerRequest *request){    /// If you get html get request for /currentlevel, respond with text: levelSens.getLitersStr().c_str()
    request->send_P(200, "text/html", levelSens.getLitersStr().c_str());
  });
  server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request){            /// /info ==> levelSens.getInfoStr().c_str()
    request->send_P(200, "text/html", levelSens.getInfoStr().c_str());
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){                /// / ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){    /// android connected to hotspot ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){          /// windows connected to hotspot ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.onNotFound([](AsyncWebServerRequest *request){                       /// request for some other webpage ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);     ///Redirect every request to captive portal
  

  server.begin();                                                             /// Start webserver

  pinMode(13, INPUT_PULLUP);                                                  /// Capacitive Sensor pin -> pull up
  attachInterrupt(13, isr, FALLING);                                          /// Capacitive pin falling edge -> trigger interrupt

  timer = timerBegin(0, 80, true);                                            /// Initialise timer interrupt for every 100ms
  timerAttachInterrupt(timer, &timerISR, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);

/// DISP INIT
  displays.init();
  displays.send2Disp(0x0f, 0x00, 0x00);  // Display mode: Test 0x01 : Normal 0x00
  displays.send2Disp(0x0c, 0x00, 0x00);   // Display : Shutdown 0x00 : On 0x01
  displays.send2Disp(0x0a, 0x01, 0x01);  // Brightness : 0x0
  displays.send2Disp(0x09, 0x00, 0x00);  // Decode mode: 0x00 - no decode, 0x01, 0x0F, 0xFF - ascii
  displays.send2Disp(0x0b, 0x07, 0x07);  // Scan Limit reg
  displays.send2Disp(0x0c, 0x01, 0x01);   // Display : Shutdown 0x00 : On 0x01
/// DISP INIT

  
}
void loop(){
  
  delay(_mainLoopDelay);           /// delay
  
  levelSens.update();              /// update display values          
  if((int)levelSens.avgFreq == 0){          /// if sensor not connected, print error to display, else print sensor readings to display
     displays.printStr("ERR     NO CON  ");
  }
  else if((int)levelSens.avgFreq >= 150000){
     displays.printStr("SENS    FAULT   ");
  }
  else{
    displays.printMenu(0,(levelSens.percent*10),levelSens.flow,levelSens.getTTedgeInt(),-1, levelSens.avgFreq,1132);
  }

  for(int i=0; i < 100; i++){
    if(!digitalRead(CALIB_SWITCH_PIN)){
      break;
    }
    delayMicroseconds(1000);
    digitalWrite(LED_GREEN, HIGH);
    levelSens.calibrate();
  }

  dnsServer.processNextRequest();   /// process DNS request if new phone is connected*/

  /*Serial.print("raw:");
  Serial.print(levelSens.getRaw());
  Serial.print(",");*/
 /* Serial.print("raw:");
  Serial.print(levelSens.getRaw());
  Serial.print(",");
  Serial.print("filtered_raw:");
  Serial.println(kFiltered(levelSens.getRaw()));*/
  /*Serial.print("filteredLiters:");
  Serial.print(levelSens.getLiters());
  Serial.print(",%:");
  Serial.print(levelSens.getPercent());
  Serial.print(",");
  Serial.print("flow:");
  Serial.println(levelSens.getFlow());*/
  
  
  //Serial.println();
}
