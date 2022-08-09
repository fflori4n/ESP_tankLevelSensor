#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "index.h"

#define EEPROM_SIZE 100 /// set this

#define _mainLoopDelay 50 /// delay for non precise timing, to limit Display and EEPROM writes

// Replace with your network credentials
const char* ssid = "*****";
const char* password = "*****";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


volatile double levelSensPulses = 0;
volatile double rawPulse = 0;
volatile double newReading = -1;

/// TIMER ISR SETUP
hw_timer_t * timer = NULL;


class LiquidLevel{
  #define _totalOutH 0
  #define _totalOutL 1
  
  private:
    double fullLiters;
    double emptyLiters;
    double fullFreq;
    double emptyFreq;

    double freq = -1;
    double dt = -1;
    double sensorFreq = -1;
    bool updateValues = false;

    double tankLiters = -1;
    double prevFilteredLiters = -1;
    double dLiters = 0;
    double filteredLiters = -1;
    double percent = -1;
    double flow = 0;
    double ttEdge = 0;

    double totalWaterOut = 0;
    double totalWaterOutFLSH = 0;
    double avrgOutFlow=0;
    unsigned int dischStartTm = 0;
    unsigned int lastFilledTime = 0;
    unsigned int lastFillDuration =0;

    float logIntervalCounter = 0;

    double litersKFiltered(double dist) {

      const double R = 10;   /// measurement noise
      const double H = 1;
      static double Q = 0.1;   /// initial covariance
      static double P = 0;    /// init. error covariance
      static double K = 0;    /// init. Kalman gain
      static double distBar = -1;     /// est. variable

      const double hist = 5;

      if (distBar == -1) {
        distBar = dist;
      }
      if(abs(distBar - dist) < hist){
        return distBar;
      }

      K = P * H / (H * H * P + R);
      distBar = distBar + K * (dist - H * distBar);
      P = (1 - K * H) * P + Q;
      return distBar;
    }
    double flowKFiltered(double dist) {

      const double R = 100;   /// measurement noise
      const double H = 1;
      static double Q = 0.1;   /// initial covariance
      static double P = 0;    /// init. error covariance
      static double K = 0;    /// init. Kalman gain
      static double distBar = -1;     /// est. variable

      //const double hist = 5;

      if (distBar == -1) {
        distBar = dist;
      }
      /*if(abs(distBar - dist) < hist){
        return distBar;
      }*/

      K = P * H / (H * H * P + R);
      distBar = distBar + K * (dist - H * distBar);
      P = (1 - K * H) * P + Q;

      if(distBar > 1000){
        return 1000;
      }
      return distBar;
    }
    double calcRunningAvrg(double newSensReading){
      const int _NumOfHistory = 150;         /// array for average filtering
      static double lastNreadings[_NumOfHistory];
      static double average = 0;
      double newAverage = 0;

      for(int i=0; i< (_NumOfHistory -1); i++){
        lastNreadings[i] = lastNreadings[i+1];
        newAverage+=lastNreadings[i+1];
      }
      lastNreadings[(_NumOfHistory -1)] = newSensReading;
      if(abs(newAverage/(_NumOfHistory-1) - newSensReading) > 0){
        average = ((newAverage/(_NumOfHistory-1)) + newSensReading)/2;
      }
      else{
        newAverage+= newSensReading;
        newAverage/=_NumOfHistory;
        average = newAverage;
      }
      
      return average;
    }
  public:

  LiquidLevel(double fullLiters, double emptyLiters, double fullFreq, double emptyFreq){
    this->fullLiters = fullLiters;
    this->emptyLiters = emptyLiters;
    this->fullFreq = fullFreq;
    this->emptyFreq = emptyFreq;
  }
  void setFreq(double freq, double dt){
    /// Keep this short, runs in ISR
    this->freq = freq;
    this->dt = dt;
    updateValues = true;
    logIntervalCounter += (dt*1.0)/1000;
  }
  void saveToEEPROM(){
    #define _maxLogRate 1
    #define _minLogRate 60
    #define _periodicLog false
    static unsigned int minLogInterval = _maxLogRate * 60;

    unsigned int logLimit = constrain(map(abs(flow),400,0,_maxLogRate * 60,_minLogRate * 60),_maxLogRate * 60,_minLogRate * 60);
    

    minLogInterval = min(minLogInterval, logLimit);
    
    /*Serial.print(logLimit);
    Serial.print(" ");
    Serial.print(minLogInterval);
    Serial.print(" ");
    Serial.println(logIntervalCounter);*/
    
    if((logIntervalCounter > minLogInterval) && (abs(totalWaterOut - totalWaterOutFLSH) > 0.005*2000)){
      logIntervalCounter = 0;
      minLogInterval = _minLogRate * 60;
      Serial.println("LOG LOG");

      totalWaterOutFLSH = totalWaterOut;
      Serial.print("writing total Out to EEPROM: ");
      Serial.println(totalWaterOut);
      EEPROM.write(_totalOutL, ((uint16_t)totalWaterOut >> 0) & 0xFF);
      EEPROM.write(_totalOutH, ((uint16_t)totalWaterOut >> 8) & 0xFF);
      EEPROM.commit();
    }
    /*static long int lastWrite = 0;
   // Serial.println("EEPROM LOG:");
   double totalOutDiff = abs(totalWaterOut - totalWaterOutFLSH);
   double logInterval = 1 + (1/(abs(flow)+1)) * 20;
   Serial.println(logInterval);
   
   
    if(( totalOutDiff > (0.05*2000) && lastWrite > (_maxChangeWriteRate * 3000/_mainLoopDelay))){
      totalWaterOutFLSH = totalWaterOut;
      Serial.print("writing total Out to EEPROM: ");
      Serial.println(totalWaterOut);
      EEPROM.write(_totalOutL, ((uint16_t)totalWaterOut >> 0) & 0xFF);
      EEPROM.write(_totalOutH, ((uint16_t)totalWaterOut >> 8) & 0xFF);
      EEPROM.commit();
    }
    lastWrite++;
    Serial.println(lastWrite);*/
      
  }
  void loadEEPROM(){
    uint16_t totalWaterOut = (EEPROM.read(_totalOutH) << 8) | (EEPROM.read(_totalOutL) & 0xff);
    this->totalWaterOut = totalWaterOutFLSH = totalWaterOut;
    Serial.print("totalWater out from EEPROM: ");
    Serial.println(totalWaterOut);
  }
  void lutLevel(double in){
    /*double pulseLUT[] = {16500, 17500,  18860, 20190, 21760,  23440,}
    double levelLUT[] = {1020,  1000,     900,   800,   700,    600,}*/

    /// 20000
    double out = 0;
    double LUT[] = {
      16500,1020,
      17500,1000,
      18860,900,
      20190,800,
      21760,700,
      23440,600,
      25500,500,
      27670,402,
      30770,300,
      32200,200,
      38980,100,
      44000,0
    };
    if(in > LUT[(sizeof(LUT) / sizeof(LUT[0]))-2]){
      out = LUT[(sizeof(LUT) / sizeof(LUT[0]))-1];
    }
    else if(in < LUT[0]){
      out = LUT[1];
    }
    for(int i=0; i< ((sizeof(LUT) / sizeof(LUT[0]))-2); i+=2){
      if(in > LUT[i] && in <= LUT[i+2]){
        double percent = ((in - LUT[i])/(LUT[i+2] - LUT[i]));

        out = LUT[i+1] +((LUT[i+3] - LUT[i+1]) * percent);
        //out = LUT[i+1];
        break;
      }
    }
    Serial.print("Sensor depth: ");
    Serial.println(out);
    }
  void calcLevel(){
    #define _dLiterHist 0
    
    if(!updateValues){
      return;
    }
    updateValues = false;
    //Serial.println(freq);
    /// Linear approximation, works for this tank because of shape, might need LUT for complex cross sections
    lutLevel(this->freq);
    tankLiters = constrain(map(this->freq, emptyFreq, fullFreq, emptyLiters, fullLiters),emptyLiters, fullLiters);
    if(prevFilteredLiters == -1){
      prevFilteredLiters = tankLiters;
      
    }
    else{
      prevFilteredLiters = filteredLiters;
    }
    filteredLiters = litersKFiltered(tankLiters);
    percent = ((filteredLiters-emptyLiters) *100)/ (fullLiters-emptyLiters);
    dLiters = prevFilteredLiters - filteredLiters;
    if(abs(dLiters) > _dLiterHist){
      if(dLiters > _dLiterHist){
        totalWaterOut += dLiters;
      }
      this->saveToEEPROM();
    }  
    flow = (dLiters)*10*60;
    flow = flowKFiltered(calcRunningAvrg(flow));//flowKFiltered(flow);
  }
  double getRaw(){
    return freq;
  }
  double getLiters(){
    return filteredLiters;
  }
  String getLitersStr(){
    return String(filteredLiters);
  }
  double getPercent(){
    return percent;
  }
  double getFlow(){
     return flow;
  }
  String getInfoStr(){
    /// TankLevel/ TankLiters/ Flow/ TMUntilEmpty/ SpentWater/ AverageFlow/ DischStarted/ Last fill time/ 
    return String(this->percent) + "/" + String(this->filteredLiters) + "/" + String(this->flow) + "/" +String(this->ttEdge) + "/" +String(this->totalWaterOut) +"/" + String(this->avrgOutFlow)+ "/" + String(this->dischStartTm)+"/" + String(this->lastFilledTime)+ "/" + String(this->lastFillDuration);  
  }
  void setTotalOut(double totalOut){
    this->totalWaterOut = totalOut;
    }
};

LiquidLevel levelSens(2000, 0, 16500, 46000);


String getCurrentLevel() {
  /*static int counter = 0;
  counter++;
  return String(counter);*/
  return String(rawPulse);
}
String getInfo(){
  /// TankLevel/ TankLiters/ Flow/ TMUntilEmpty/ SpentWater/ AverageFlow/ DischStarted/ Last fill time/ 
 // String webDispStr = String(levelSens.getPercent()) + "/" + String( levelSens.getLiters()) + "/20/5/120000/20/1222/55644/2234";
  return levelSens.getInfoStr();
}

void IRAM_ATTR isr() {
  levelSensPulses++;
}
void IRAM_ATTR timerISR() {
  newReading = rawPulse = levelSensPulses;
  levelSens.setFreq((levelSensPulses*10), 100);
  //Serial.println(levelSensPulses);
  levelSensPulses = 0;
}
double kFiltered(double dist) {

  const double R = 1000;//10;   /// measurement noise
  const double H = 1;
  static double Q = 0.1;   /// initial covariance
  static double P = 0;    /// init. error covariance
  static double K = 0;    /// init. Kalman gain
  static double distBar = -1;     /// est. variable

  const double hist = 5;

  if (distBar == -1) {
    distBar = dist;
  }
  if(abs(distBar - dist) < hist){
    return distBar;
  }

 /* if (dist < MIN_TANK_HEIGHT || dist > MAX_TANK_HEIGHT) {
    return distBar;
  }*/

  K = P * H / (H * H * P + R);
  distBar = distBar + K * (dist - H * distBar);
  P = (1 - K * H) * P + Q;
  return distBar;
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  levelSens.loadEEPROM();
 // levelSens.setTotalOut(2000);
  //levelSens.saveEEPROM();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1500);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  server.on("/currentLevel", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", levelSens.getLitersStr().c_str());
  });
  server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", getInfo().c_str());
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", String(indexStr).c_str());
  });

  // Start server
  server.begin();

  pinMode(13, INPUT_PULLUP);
  attachInterrupt(13, isr, FALLING);
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerISR, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);

  
}
 
void loop(){
  
  /*Serial.print(calcSigma());
  Serial.print(",");
  Serial.print("filtered:");
  Serial.println(kFiltered(calcSigma()));*/
  delay(_mainLoopDelay);
  levelSens.calcLevel();

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
