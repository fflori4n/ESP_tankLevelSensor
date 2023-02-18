/**   KalmanFilter object
 *    create a Kalman filter with custum parameters, and filter
 *    the sensor reading
 */
class KalmanFilter{
  private:
    double R = 10;   /// measurement noise
    double H = 1;

    double Q = 0.1;   /// initial covariance
    double P = 0;    /// initial error covariance
    double K = 0;    /// initial Kalman gain

    const double hist = 5;
  public:
    double distBar = -1;     /// estimated variable

  KalmanFilter(){            /// empty constructor
   // this->R = 1;
   // this->H = 10;
    //this->R = R;
    //this->H = H;
  }
  void setRH(double R, double H){   /// set parameters of Kalman filter
    this->R = R;
    this->H = H;
  }
  double getFilt(double dist){     /// get filtered
    if (distBar == -1) {           /// if this is the first reading, do not filter
      distBar = dist;
    }
    if(abs(distBar - dist) < hist){   /// if the difference between estimated and real is less than hist, do not filter
      return distBar;                 ///return previous filtered
    }

    K = P * H / (H * H * P + R);                      /// do the Kalman filter magic.
    distBar = distBar + K * (dist - H * distBar);
    P = (1 - K * H) * P + Q;

    return distBar;                   /// return filtered
  }
};


class LiquidLevel{
  /// define memory locations in EEProm to contain total out and average flow variables
  #define _totalOutH 0
  #define _totalOutL 1
  #define _avgOutFlowH 2
  #define _avgOutFlowL 3

  /// flags for printing debug messeges 
  #define PRINT_RAWFREQ 1
  //#define PRINT_STATUS 1
  //#define PRINT_LOGDBG 1
  //#define PRINT_STATE
  
  private:
    
  public:

    bool noConErr = false;        /// sensor not connected error

    double fullLiters;
    double emptyLiters;
    double fullFreq;
    double emptyFreq;

    double freq = -1;
    double avgFreq = -1;
    double dt = -1;
    double sensorFreq = -1;
    
    double dLiters = 0;
    double filteredLiters = -1;
    double percent = -1;
    bool levelCalcRDY = false;

    double flowDt = 1;
    double flowNowFreq = 0;
    double flowNowLiters = 99999999;
    double flowPrevLiters = 99999999;
    bool flowCalcRDY = false;
    
    double flow = 0;
    double ttEdge = 0;
    double ttEdgeAvgOpen = 0;

    double totalWaterOut = 0;
    double totalWaterOutFLSH = 0;
    double avrgOutFlow=0;

    unsigned int dischStartTm = 0;
    unsigned int lastFilledTime = 0;
    unsigned int lastFillDuration =0;

    unsigned int flowStatus = 0; /// 0 - STATIC, 1 - OUT, 2 - IN
    unsigned int levelStatus = 3; /// 0 - EMPTY, 1 - LOW, 2 - MID, 3 - HIGH, 4 - FULL

    float logIntervalCounter = 0;

    float tmCycle = 0;

    float tmSinceLastFill = 0;
    float tmSinceLastSprayStart = 0;
    float tmSinceLastOpen = 0;

    KalmanFilter levelLitersFilter;

    /**
     * @brief Constructor for liquid level sensor object
     * 
     * @param fullLiters num of liters when reservoir is full
     * @param emptyLiters num of liters when reservoir is empty
     * @param fullFreq frequency when reservoir is full
     * @param emptyFreq frequency when reservoir is empty
     * 
     */
    LiquidLevel(double fullLiters, double emptyLiters, double fullFreq, double emptyFreq){
      this->fullLiters = fullLiters;
      this->emptyLiters = emptyLiters;
      this->fullFreq = fullFreq;
      this->emptyFreq = emptyFreq;

      levelLitersFilter.setRH(10,1);
    }

    /**
     * @brief dt time has passed and freq number of pulses was counted, time to update sensor readings
     * 
     * @param freq 
     * @param dt 
     * 
     *  /// Keep this short, runs in ISR
     */
    void setFreq(double freq, double dt){
      #define AVGWEIGHT 0.95                                              /// weight of previous measurement in average filtering
      #define FLOW_MEASURE_INTERVAL 1000                                  /// log variables for flow calculation every  1000ms
      static double calcFlowDt = 0;
      this->freq = freq;
      this->dt = dt; 

      if(this->avgFreq == -1 && this->freq != -1){                        /// at initial measurement, use new value as average value
        this->avgFreq = this->freq;
      }
      this->avgFreq = ((avgFreq * AVGWEIGHT) + (freq * (1 - AVGWEIGHT))); /// weighted average filtering of new value and average value
      levelCalcRDY = true;                                                /// set flag, new values have been recorded time to recalculate level

      
      calcFlowDt+= dt;                                                    /// increase time passed since last flow calc.
      if(calcFlowDt >= FLOW_MEASURE_INTERVAL){                            /// Check if enough time has passed, to rerecord measurements for flow calc.
        this->flowDt = calcFlowDt;
        this->flowNowFreq = this->avgFreq;
        this->flowCalcRDY = true;
        calcFlowDt = 0;
      }
      logIntervalCounter += (dt*1.0)/1000;                                /// increase time passed since last EEPROM log

      //tmCycle += (dt*1.0)/1000;
    }

    void calibrate(){
      #define _SAMPLE_MS 1500

      static int calibrationStep = 0;
      int msToStart = 5000;
      static int64_t tValveOpened, prevEspTime;
      
      Serial.print("calibrating sensor...");
      Serial.print("open valve in:");

      while(true){
        digitalWrite(LED_GREEN, HIGH);
        if(calibrationStep == 0){
          digitalWrite(LED_BLUE, HIGH);
        }
        delay(10);
        this-> update(false);

        switch(calibrationStep){
          case 0:
            msToStart-= 10;
            if(msToStart%100 == 0){
              Serial.println(msToStart/100);
            }
            if(msToStart<= 0){
              calibrationStep++;
              tValveOpened = esp_timer_get_time();
            }
            break;
          case 1:
            int64_t now = esp_timer_get_time();
            if((now - prevEspTime) >= (1000 * _SAMPLE_MS)){  /// every 100 ms
              //Serial.print("t+,fq: ");
              prevEspTime = now;
              Serial.print((now - tValveOpened)/1000);
              Serial.print(",");
              Serial.println((int)this->freq);
            }
            if(digitalRead(CALIB_SWITCH_PIN) == 0){
              digitalWrite(LED_GREEN, LOW);
              Serial.println("calibration mode exit.");
              delay(10000);
              return;
            }
            break;
        }
      }
    }
    /*void calibrate(){
      
      
      
      while(true){
       // Serial.println("calib mode");
        digitalWrite(LED_GREEN, HIGH);
        delay(10);
        this-> update(false);

        switch(calibrationStep){
          case 0:
            msToStart-= 10;
            if(msToStart%100 == 0){
              Serial.println(msToStart/100);
            }
            if(msToStart<= 0){
              calibrationStep++;
              tValveOpened = esp_timer_get_time();
            }
            break;
          case 1:
            int64_t now = esp_timer_get_time();
            if((now - prevEspTime) >= (1000 * _SAMPLE_MS)){  /// every 100 ms
              //Serial.print("t+,fq: ");
              prevEspTime = now;
              Serial.print((now - tValveOpened)/1000);
              Serial.print(",");
              Serial.println((int)currentFreq);

            for(int i=0; i < 100; i++){
              if(digitalRead(CALIB_SWITCH_PIN)){
                break;
              }
              delayMicroseconds(1000);
            }
            if(digitalRead(CALIB_SWITCH_PIN)){
              digitalWrite(LED_GREEN, LOW);
              Serial.println("calibration mode exit.");
            }
            
            break;
          case 2:
            break;
          default:
            break;
        }
    }*/
  
    /**
    * @brief Write 'total water out' and 'average flow' to non volatile EEPROM, these values remain even after microcontroller power off
    * Log interval changes depending on flow values, if flow is small, log period long, - if flow big, log period small
    */
    void saveToEEPROM(){
      return;
      #define _maxLogRate 1
      #define _minLogRate 60
      #define _periodicLog false

      static unsigned int minLogInterval = _maxLogRate * 60;

      unsigned int logLimit = constrain(map(abs(flow),150,0,_maxLogRate * 60,_minLogRate * 60),_maxLogRate * 60,_minLogRate * 60);
      

      minLogInterval = min(minLogInterval, logLimit);
      
      #ifdef PRINT_LOGDBG
      Serial.print(logLimit);
      Serial.print(" ");
      
      
      Serial.print(logIntervalCounter);
      Serial.print(" > ");
      Serial.println(minLogInterval);
      #endif
      
      if((logIntervalCounter > minLogInterval)){
        logIntervalCounter = 0;
        minLogInterval = _minLogRate * 60;
        Serial.println("LOG LOG");

        totalWaterOutFLSH = totalWaterOut;
        Serial.print("writing total Out to EEPROM: ");
        Serial.println(totalWaterOut);
        if((abs(totalWaterOut - totalWaterOutFLSH) > 0.005*2000)){
          EEPROM.write(_totalOutL, ((uint16_t)totalWaterOut >> 0) & 0xFF);
          EEPROM.write(_totalOutH, ((uint16_t)totalWaterOut >> 8) & 0xFF);
          this->totalWaterOutFLSH = this->totalWaterOut;
        }
        

        EEPROM.write(_avgOutFlowL, ((uint16_t)avrgOutFlow >> 0) & 0xFF);
        EEPROM.write(_avgOutFlowH, ((uint16_t)avrgOutFlow >> 8) & 0xFF);
        

        EEPROM.commit();

        /// signal EEPROM write using green LED
        digitalWrite(LED_GREEN, HIGH);
        delay(100);
        digitalWrite(LED_GREEN, LOW);
        delay(100);
        digitalWrite(LED_GREEN, HIGH);
        delay(100);
      }    
    }
    /**
     * @brief Read 'total water out' and 'average flow' from non volatile EEPROM
     * 
     */
    void loadEEPROM(){
      uint16_t totalWaterOut = (EEPROM.read(_totalOutH) << 8) | (EEPROM.read(_totalOutL) & 0xff);
      this->totalWaterOut = totalWaterOutFLSH = totalWaterOut;
      Serial.println("EEPROM: ");
      Serial.print("Total water out: ");
      Serial.println(totalWaterOut);

      uint16_t avgOutFlow = (EEPROM.read(_avgOutFlowH) << 8) | (EEPROM.read(_avgOutFlowL) & 0xff);
      this->avrgOutFlow = avgOutFlow;
      Serial.print("Average flow out: ");
      Serial.println(this->avrgOutFlow);
    }
    double mapf(double x, double in_min, double in_max, double out_min, double out_max){
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * @brief Lookup table for transforming 'in' frequency on sensor pin to water level in mm-s, the relation was determined experimentally
     * 
     * @param in frequency measured on capacitive sensor pin
     * @return double water level in mm-s
     */
    double lutLevel(double in, bool verboseMode = true){
      if( in < 1){                /// if sensor reads 0Hz, probe is not connected or faulty, return error
        this->noConErr = true;
        return 0;
      }
      this->noConErr = false;
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
      /// WET PROBE TO DRY
    /*  double LUT[] = {    
        9120,1100,
        9390,1050,
        9530,1000,
        9750,950,
        9970,900,
        10240,850,
        10525,800,
        10720,750,
        10940,700,
        11320,650,
        11660,600,
        12000,550,
        12350,500,
        12670,450,
        13000,400,
        13350,350,
        13650,300,
        13900,250,
        14350,200,
        15250,150,
        15850,100,
        16500,50,
        17350,0
      };
     /* double LUT[] = {
        15480  , 999.91
15670 , 949.31
16200 , 900.02
16580 , 852.04
17080 , 805.38
17680 , 760.04
18210 , 716
18780 , 673.28
19470 , 631.8
20040 , 591.71
20820 , 552.94
21440 , 515.4
22290 , 479.26
23050 , 444.43
23850 , 410.91
24920 , 378.71
25710 , 347.82
26790 , 318.24
27710 , 289.98
29060 , 262.98
30220 , 237.35
31520 , 213.03
32950 , 190.03
34600 , 168.3
36180 , 147.93
37990 , 128.87
39380 , 111.12
42110 , 94.69
44260 , 79.58
46480 , 65.77
49010 , 53.28
51600 , 42.11
55980 , 32.24
59410 , 23.7
63590 , 16.46
67720 , 10.53
73650 , 5.93
88890 , 2.64
93840 , 0.66
94260 , 0
      };*/
      if(in > LUT[(sizeof(LUT) / sizeof(LUT[0]))-2]){ /// if measured value is bigger than bigest value in lookup table, use bigest value
        out = LUT[(sizeof(LUT) / sizeof(LUT[0]))-1];
      }
      else if(in < LUT[0]){                           /// if measured value is smaller than smallest value in lookup table, use smallest value
        out = LUT[1];
      }
      for(int i=0; i< ((sizeof(LUT) / sizeof(LUT[0]))-2); i+=2){          /// Loop over LUT, get two points next to each other
        if(in > LUT[i] && in <= LUT[i+2]){                                /// if measured value is between the two points
          double percent = ((in - LUT[i])/(LUT[i+2] - LUT[i]));           /// get the distance in % from the lower point

          out = LUT[i+1] +((LUT[i+3] - LUT[i+1]) * percent);              /// use linear approximation between points to get output
          //out = LUT[i+1];
          break;
        }
      }
      if(verboseMode){
      #ifdef PRINT_RAWFREQ
      Serial.print("Freq: ");
      Serial.print(in);
      Serial.print(" LUT Out: ");
      Serial.println(out);
      #endif
      }
      return constrain(mapf(out, 0, 1000, emptyLiters, fullLiters),emptyLiters, fullLiters);  /// convert mm-s of liquid into liters using map
    }
    /*double freqToLitersLinear(double freq){
      return constrain(map(freq, emptyFreq, fullFreq, emptyLiters, fullLiters),emptyLiters, fullLiters);
    }*/

  /**
   * @brief Update level, flow values, and detect status changes
   * 
   */
  void update(bool verboseMode = true){

    #define minFlowOUT 20
    #define minFlowIN -20
    #define minTankFull 95
    #define maxTankEMPTY 5


    if(!levelCalcRDY && !flowCalcRDY){  /// somehow there is no new reading from sensor to process
      return;
    }

    if(levelCalcRDY){                   /// There is a new reading from sensor for level - calculate water level
      levelCalcRDY = false;

      double tankLiters = lutLevel(this->avgFreq, false);                              /// get measured liters
      filteredLiters = levelLitersFilter.getFilt(tankLiters);                   /// kalman filter the liters
      percent = ((tankLiters - emptyLiters) *100) / (fullLiters - emptyLiters); /// calculate % of 'fullness'
    }
    if(flowCalcRDY){                    /// There is a new reading from sensor for flow - calculate flow
      flowCalcRDY = false;


      //flowNowLiters = freqToLitersLinear(flowNowFreq);  ///// TODO: LUT ***************************************************
      flowNowLiters = (lutLevel(flowNowFreq, verboseMode) * (fullLiters/100)); /// get measured liters

      if(this->flowPrevLiters == 99999999 && flowNowLiters != 99999999){    /// if this is the first measurement, use this value as previous value
        this->flowPrevLiters = flowNowLiters;
      }
      dLiters = this->flowPrevLiters - flowNowLiters; /// calculate difference between previous liters and current liters = flow [liters/1sec]
      flowPrevLiters = flowNowLiters;

      double newFlow = (dLiters / this-> flowDt) * 1000 * 60; /// (difference in liters / time of flow sampling) * 1000ms * 60 = [l/min]

      //this->flow = calcRunningAvrg(newFlow);

                                            /// apply non uniform filtering, idea like AIMD algorithm, flow can decrease fast, but can only increase slowly 
      if(abs(newFlow) < abs(this->flow)){   /// if flow is decreasing filter less
        this->flow = 0.8 * newFlow;
      }
      else{                                 /// if flow is increasing, filter more
        this->flow =(0.9 * this->flow) + (0.005 * newFlow); //(0.80 * this->flow) + (0.2 * newFlow) ;
      }
      
    }


    if(flow > minFlowIN && flow < minFlowOUT){            /// flow is close to zero, level is more or less static
      flowStatus = 0;                                     /// 0 - STATIC, 1 - OUT, 2 - IN
      this->ttEdge = -1;                                  /// time until tank empty/ or full is infinity
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, LOW);
      #ifdef PRINT_STATUS
      Serial.print(" Flow: Static ");
      #endif
    }
    else if(flow > minFlowOUT){                                                 /// flow is not static - water level decreasing
        this->avrgOutFlow = (this->avrgOutFlow * 0.8) + (this->flow * 0.2);     /// calculate average flow out - only if level is descending AND save to EEPROM
        this->ttEdge = abs((emptyLiters - filteredLiters) / flow);              /// calculate estimated time until reservoir is empty based on current flow
        this->ttEdgeAvgOpen = abs((emptyLiters - filteredLiters)/this->avrgOutFlow);/// calculate estimated time until reservoir is empty based on average flow
        flowStatus = 1;                                                         /// 0 - STATIC, 1 - OUT, 2 - IN

        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);
        #ifdef PRINT_STATUS
        Serial.print(" Flow: Out    ");
        #endif
    }
    else{                                                                   /// flow is not static - water level increasing
        this->ttEdge = abs((fullLiters - filteredLiters) / flow);           /// calculate estimated time until reservoir is full based on current flow
        flowStatus = 2;                                                     /// 0 - STATIC, 1 - OUT, 2 - IN
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);
        #ifdef PRINT_STATUS
        Serial.print(" Flow: In    ");
        #endif
    }

    this->saveToEEPROM();   /// run method for logging to EEPROM if needed


    /// TANK STATUS STATE
    /// 0 - EMPTY, 1 - LOW, 2 - MID, 3 - HIGH, 4 - FULL

    #define EMPTY_LOW 5
    #define LOW_MID 25
    #define MID_HIGH 60
    #define HIGH_FULL 95

    /// detect when level is crossing certain points, standard state machine - syntax like PLC code.
    if(levelStatus == 0 ){            /// EMPTY
      if(this->percent > EMPTY_LOW){
        levelStatus = 1;
        #ifdef PRINT_STATE
        Serial.println(" Tank: EMPTY - LOW - RISING ");
        #endif
      }
    }
    else if(levelStatus == 1){        /// LOW
      if(this->percent <= EMPTY_LOW){
        levelStatus = 0;
        #ifdef PRINT_STATE
        Serial.println(" Tank: EMPTY - LOW - FALLING ");
        #endif
      }
      else if(this->percent > LOW_MID){
        levelStatus = 2;
        #ifdef PRINT_STATE
        Serial.println(" Tank: LOW - MID - RISING ");
        #endif
      }
    }
    else if(levelStatus == 2){      /// MID
      if(this->percent <= LOW_MID){
        levelStatus = 1;
        #ifdef PRINT_STATE
        Serial.println(" Tank: LOW - MID - FALLING ");
        #endif

      }
      else if(this->percent > MID_HIGH){
        levelStatus = 3;
        #ifdef PRINT_STATE
        Serial.println(" Tank: MID - HIGH - RISING ");
        #endif
      }
    }
    else if(levelStatus == 3){    /// HIGH
      if(this->percent <= MID_HIGH){
        levelStatus = 2;
        #ifdef PRINT_STATE
        Serial.println(" Tank: MID - HIGH - FALLING ");
        #endif
      }
      else if(this->percent > HIGH_FULL){
        levelStatus = 4;
        #ifdef PRINT_STATE
        Serial.println(" Tank: HIGH - FULL - RISING ");
        #endif
      }
    }
    else if( levelStatus == 4){   /// FULL
      if(this->percent <= HIGH_FULL){
        levelStatus = 3;
        #ifdef PRINT_STATE
        Serial.println(" Tank: HIGH - FULL - FALLING ");
        #endif
      }

    }

    if(verboseMode){
    #ifdef PRINT_STATUS
    Serial.print(" levelPerc: ");
    Serial.print(percent);
    Serial.print(" Dliters: ");
    Serial.print(dLiters);
    Serial.print(" Flow: ");
    Serial.print(flow);
    Serial.println(" ");
    #endif
    }
  }
  /*void calcLevel(){
    #define _dLiterHist 0
    
    
    //updateValues = false;
    //Serial.println(freq);
    /// Linear approximation, works for this tank because of shape, might need LUT for complex cross sections
    lutLevel(this->freq);
    tankLiters = constrain(map(this->freq, emptyFreq, fullFreq, emptyLiters, fullLiters),emptyLiters, fullLiters);  ///// TODO: LUT ***************************************************
    if(prevFilteredLiters == -1){
      prevFilteredLiters = tankLiters;
      
    }
    else{
      prevFilteredLiters = filteredLiters;
    }
    filteredLiters = litersKFiltered(tankLiters);
    
    /*flow = (dLiters)*10*60;
    /*flow = flowKFiltered(calcRunningAvrg(flow));//flowKFiltered(flow);
    if(flowCalcRDY){
      flow = calcFlow();
    }
  }*/
  /**
   * @brief get estimated time until resorvoir empty or full in display format
   * 
   * 1xxxx - blank letter A
   * x11xx - minutes until empty
   * xxx11 - seconds until empty
   * @return int 
   */
  int getTTedgeInt(){
   int returnInt = 0;
   int ttEdgeSeconsd = this->ttEdge*60; /// ttEdge seconds = ttEdge (type double in minutes) * 60

   if(this->ttEdge == -1){              /// level is static - time until empty or full is infinity, use average open flow to calculate estimated time.
    ttEdgeSeconsd = this->ttEdgeAvgOpen * 60;
   }
   else{
    returnInt += 10000; /// blank letter A to indicate current flow, not average open flow
   }
   returnInt += ttEdgeSeconsd%60;
   returnInt += (ttEdgeSeconsd/60)*100;

   return returnInt;  
  }
  /**
   * @brief Return filtered liters string for displaying purposes
   * 
   * @return String 
   */
  String getLitersStr(){
    return String(filteredLiters);
  }
  /**
   * @brief Return string with current measurement values
   * 
   * @return String 
   */
  String getInfoStr(){
    /// TankLevel/ TankLiters/ Flow/ TMUntilEmpty/ SpentWater/ AverageFlow/ DischStarted/ Last fill time/ 
    return String(this->percent) + "/" + String(this->filteredLiters) + "/" + String(this->flow) + "/" +String(this->ttEdge) + "/" +String(this->totalWaterOut) +"/" + String(this->avrgOutFlow)+ "/" + String(this->dischStartTm)+"/" + String(this->lastFilledTime)+ "/" + String(this->lastFillDuration);  
  }
  /**
   * @brief Set the Total Out object
   * 
   * @param totalOut 
   */
  void setTotalOut(double totalOut){
    this->totalWaterOut = totalOut;
    }
};
