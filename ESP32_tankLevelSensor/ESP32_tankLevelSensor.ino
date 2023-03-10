#define EEPROM_SIZE 100             /// EEPROM size, non volatile memory for storing averages and statistics when power is cut
#define _mainLoopDelay 50           /// delay for non precise timing, to limit Display and EEPROM writes
#define CAP_SENS_PIN 13             /// Capacitive sensor pin
#define LED_BLUE 32                 /// blue LED pin
#define LED_GREEN 33                /// green LED pin
#define CALIB_SWITCH_PIN 19         /// switch for going into calibration mode

#define ENCODER_A_PIN 18
#define ENCODER_B_PIN 5
#define ENCODER_BTN 17

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
//#include <SPIFFS.h>
#include <EEPROM.h>

#include <DNSServer.h>          /// needed for captive AP
#include <AsyncTCP.h>


#include "index.h"              /// file containing html for web interface
#include "kFilt.h"
#include "lutFreqTransform.h"
#include "max7219.h"            /// containing methods for displaying stuff on 8x 7seg display
#include "liquidLevel.h"        /// containing liquid level class for processing cap. sensor data

const char* hotspotSSID = "Water level sensor";   /// ESP hotspot name
const char* hotspotPasswd = "123456789";          /// ESP hotspot passwd

DNSServer dnsServer;                              /// create DNS server
AsyncWebServer server(80);                        /// create webserver on port 80 - AsyncWebServer

Display8x8 displays;

volatile double levelSensPulses = 0;          /// pulses measured from capacitive sensor
volatile double rawPulse = 0;
volatile double newReading = -1;

volatile int encoderInc = 0;
volatile int buttonPressed = 0;

int dispMenuNumber = 0;
bool editMode = false;
byte displayPages = 7;

/// TIMER ISR SETUP
hw_timer_t * timer = NULL;

LiquidLevel levelSens(2000, 0); /// new instance of liquid level class for capacitive sensor

class CaptiveRequestHandler : public AsyncWebHandler {  /// Captive access point class
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request) {
      //request->addInterestingHeader("ANY");
      return true;
    }

    void handleRequest(AsyncWebServerRequest *request) {
      /*Serial.print("request for: ");
        Serial.print(request->host().c_str());
        Serial.print(" ");
        Serial.println(request->url().c_str());*/
      request->send(200, "text/html", String(indexStr).c_str());
    }
};

void IRAM_ATTR isr() {        /// interrupt service routine, executerd on every sensor pin falling edge
  levelSensPulses++;          /// increase pulse count
}
void IRAM_ATTR timerISR() {   /// timer interrupt routine - executed every 100ms

  newReading = rawPulse = levelSensPulses;  /// TODO: ? this is probably not needed
  levelSens.setFreq((levelSensPulses * 10), 100); /// pass number of pulses in the last 100ms to function
  levelSensPulses = 0;
}
void encoderCallback() {
  static byte prevEncoder = 0;
  noInterrupts();

  byte encoder = ((digitalRead(ENCODER_B_PIN) && 0b00000001) << 1) | (digitalRead(ENCODER_A_PIN) && 0b00000001);

  // 1320 0231
  switch (encoder) {
    case 1:
      if (prevEncoder == 0) {
        //Serial.println("Encoder right");  /// should not be printing in ISR but didn't crash, so guess I can?
        encoderInc--;
      }
      else if (prevEncoder == 3) {
        //Serial.println("Encoder left");
        encoderInc++;
      }
      break;
  }
  prevEncoder = encoder;
  interrupts();
}
int readVpVoltage() {

#define VPPIN 36
#define VP_SCALE 12.26/1182  /// real voltage / ADC value

  static int vp = 1350;
  vp = (vp * 0.95) + ((VP_SCALE * analogRead(VPPIN) * 100) * 0.05);
  /*Serial.print("VBAT| ");
    Serial.println(vp);
    Serial.print("ADC_BAT| ");
    Serial.println(analogRead(VPPIN));*/
  return vp;
}


void setup() {
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

  /*server.on("/currentLevel", HTTP_GET, [](AsyncWebServerRequest *request){    /// If you get html get request for /currentlevel, respond with text: levelSens.getLitersStr().c_str()
    request->send_P(200, "text/html", levelSens.getLitersStr().c_str());
    });*/
  server.on("/info", HTTP_GET, [](AsyncWebServerRequest * request) {          /// /info ==> levelSens.getInfoStr().c_str()
    request->send_P(200, "text/html", levelSens.getInfoStr().c_str());
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {              /// / ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest * request) {  /// android connected to hotspot ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest * request) {        /// windows connected to hotspot ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.onNotFound([](AsyncWebServerRequest * request) {                     /// request for some other webpage ==> String(indexStr).c_str() /// serve the index page stored in index.h
    request->send(200, "text/html", String(indexStr).c_str());
  });
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);     ///Redirect every request to captive portal
  server.begin();                                                             /// Start webserver

  pinMode(13, INPUT_PULLUP);                                                  /// Capacitive Sensor pin -> pull up
  attachInterrupt(13, isr, FALLING);                                          /// Capacitive pin falling edge -> trigger interrupt

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  attachInterrupt(ENCODER_A_PIN, encoderCallback, CHANGE); // Need to detect both rising or falling signal
  attachInterrupt(ENCODER_B_PIN, encoderCallback, CHANGE);
  attachInterrupt(ENCODER_BTN, encoderCallback, CHANGE);

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
bool confirmAction(){

  byte twists = 1;
  #define TIME_OUT_MS 2000
  for(int i=0; i< TIME_OUT_MS; i++){
    if(encoderInc > 0){
      twists+=encoderInc;}
    encoderInc = 0;
    displays.printConfirm(twists);
    delayMicroseconds(1000);
    if(twists > 8){
      delayMicroseconds(100000);
      encoderInc = 0;
      return true;
    }
  }
  return false;
}
void loop() {

  delay(_mainLoopDelay);           /// delay
  levelSens.update();              /// update display values
  readVpVoltage();
  displays.printMenu(dispMenuNumber, (levelSens.percent * 10), levelSens.flow, levelSens.getTTedgeInt(), -1, levelSens.avgFreq, readVpVoltage(), (int)(lut1Outputmms), (int)(lut2Outputmms));

  
  if (digitalRead(CALIB_SWITCH_PIN)) {  /// switch for calibration
    digitalWrite(LED_GREEN, HIGH);
    levelSens.calibrate();
  }

  if (!digitalRead(ENCODER_BTN)) {      /// button press for navigation
    int buttonPressCounter = 0;
    while (!digitalRead(ENCODER_BTN) && buttonPressCounter < 1000) {
      buttonPressCounter += 1;
      delayMicroseconds(1000);
    }
    if (buttonPressCounter < 500) {
      buttonPressed += 1;
    }
    else {
      buttonPressed += 2;
    }
  }

  if(buttonPressed != 0){
    switch(dispMenuNumber){
      case 2: /// clear total
        if(buttonPressed == 2){
          Serial.println("confirm log clear!");
          if(confirmAction()){
            Serial.println("clear");
          }
          else{
            Serial.println("no clear");
          }
        }
        break;
      case 6:
        if(buttonPressed == 2){
          if(confirmAction()){
            if(displayPages == 9){
              displayPages = 7;
              Serial.println("debug mode disabled!");
            }
            else{
              displayPages = 9;
              Serial.println("debug mode enabled!");
            }
          }
        }
        break;
    }
    buttonPressed = 0;
  }
  if (!editMode) {
    if (encoderInc != 0) {
      if ( (dispMenuNumber + encoderInc) >= 0 && (dispMenuNumber + encoderInc) < displayPages) {
        dispMenuNumber += encoderInc;
        Serial.print(dispMenuNumber);
      }
      encoderInc = 0;
    }
  }

  dnsServer.processNextRequest();   /// process DNS request if new phone is connected*/
}
