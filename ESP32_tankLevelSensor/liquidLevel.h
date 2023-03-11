
class LiquidLevel{
  /// define memory locations in EEProm to contain total out and average flow variables
  /// flags for printing debug messeges 
  #define PRINT_RAWFREQ 1
  //#define PRINT_STATUS 1
  //#define PRINT_LOGDBG 1
  //#define PRINT_STATE

  #define SENS_BOTTOM_OFFSET 0
  #define SENS_TOP_OFFSET 0
  #define MAX_LITERS 2000
  #define SENS_MAX_LENGTHmm 1040
  
  private:
    
  public:

    bool noConErr = false;        /// sensor not connected error

    double submergedPercent = -1;
    double litersInReservoir = -1;
    double prevLitersInReservoir = -1;
    double sensorSubmergedmm = 0;

    //double fullLiters;
    //double emptyLiters;
    //double fullFreq;
    //double emptyFreq;
    
    double rawFreq = -1;
    double filteredFreq = -1;
    double freqDt = -1;

    double dLiters = 0;
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

   // double totalWaterOut = 0;
   // double totalWaterOutFLSH = 0;
    double avrgOutFlow=0;

   double totalWaterUsed = 0;
   double measuredWaterUsed = 0;
   double averageFallingFlow = 0;
   double averageRisingFlow = 0;
   

    unsigned int flowStatus = 0; /// 0 - STATIC, 1 - OUT, 2 - IN
    unsigned int levelStatus = 3; /// 0 - EMPTY, 1 - LOW, 2 - MID, 3 - HIGH, 4 - FULL

    float logIntervalCounter = 0;
    float tmCycle = 0;

    KalmanFilter levelLitersFilter;
    //KalmanFilter flowFilter;

    LiquidLevel(double fullLiters, double emptyLiters){
      //this->fullLiters = fullLiters;
      //this->emptyLiters = emptyLiters;

      levelLitersFilter.setRH(10,1);
      //flowFilter.setRH(5,1);
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

      
      this->rawFreq = freq;
      this->freqDt = dt; 

      if(this->filteredFreq == -1 && freq != -1){                        /// at initial measurement, use new value as average value
        this->filteredFreq = freq;
      }
      this->filteredFreq = ((filteredFreq * AVGWEIGHT) + (freq * (1 - AVGWEIGHT))); /// weighted average filtering of new value and average value
      levelCalcRDY = true;                                                /// set flag, new values have been recorded time to recalculate level

      
      calcFlowDt+= dt;                                                    /// increase time passed since last flow calc.
      if(calcFlowDt >= FLOW_MEASURE_INTERVAL){                            /// Check if enough time has passed, to rerecord measurements for flow calc.
        this->flowDt = calcFlowDt;
        this->flowNowFreq = this->filteredFreq;
        this->flowCalcRDY = true;
        calcFlowDt = 0;
      }
      logIntervalCounter += (dt*1.0)/1000;                                /// increase time passed since last EEPROM log

      //tmCycle += (dt*1.0)/1000;
    }

    void calibrate(){
      #define _SAMPLE_MS 500

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
              Serial.print("0");
              Serial.print(",");
              Serial.println((int)this->rawFreq);
              calibrationStep++;
              tValveOpened = esp_timer_get_time();
            /*}*/
            break;
          case 1:
            int64_t now = esp_timer_get_time();
            if((now - prevEspTime) >= (1000 * _SAMPLE_MS)){  /// every 100 ms
              prevEspTime = now;
              Serial.print((now - tValveOpened)/1000);
              Serial.print(",");
              Serial.println((int)this->rawFreq);
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
   
  
    /**
    * @brief Write 'total water out' and 'average flow' to non volatile EEPROM, these values remain even after microcontroller power off
    * Log interval changes depending on flow values, if flow is small, log period long, - if flow big, log period small
    */

    #define EEPROM_TOTAL_WATER_USED_H 0
    #define EEPROM_TOTAL_WATER_USED_L 1
    #define EEPROM_MEASURED_WATER_USED_H 2
    #define EEPROM_MEASURED_WATER_USED_L 3
    #define EEPROM_FALLING_FLOW_H 4
    #define EEPROM_FALLING_FLOW_L 5
    #define EEPROM_RISING_FLOW_H 6
    #define EEPROM_RISING_FLOW_L 7
    
    void saveToEEPROM(){
      #define _maxLogRate 1
      #define _minLogRate 60
      //#define _periodicLog false
      #define PRINT_LOGDBG

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

        writeToEEprom(this->totalWaterUsed, EEPROM_TOTAL_WATER_USED_L, EEPROM_TOTAL_WATER_USED_H, 1);
        writeToEEprom(++this->measuredWaterUsed, EEPROM_MEASURED_WATER_USED_L, EEPROM_MEASURED_WATER_USED_H, 1);
        writeToEEprom(++this->averageFallingFlow, EEPROM_FALLING_FLOW_L, EEPROM_FALLING_FLOW_H, 1);
        writeToEEprom(++this->averageRisingFlow, EEPROM_RISING_FLOW_L, EEPROM_RISING_FLOW_H, 1);

        /*totalWaterOutFLSH = totalWaterOut;
        Serial.print("writing total Out to EEPROM: ");
        Serial.println(totalWaterOut);
        if((abs(totalWaterOut - totalWaterOutFLSH) > 0.005*2000)){
          EEPROM.write(_totalOutL, ((uint16_t)totalWaterOut >> 0) & 0xFF);
          EEPROM.write(_totalOutH, ((uint16_t)totalWaterOut >> 8) & 0xFF);
          this->totalWaterOutFLSH = this->totalWaterOut;
        }
        

        EEPROM.write(_avgOutFlowL, ((uint16_t)avrgOutFlow >> 0) & 0xFF);
        EEPROM.write(_avgOutFlowH, ((uint16_t)avrgOutFlow >> 8) & 0xFF);
        
total water used: 51322.00
measured water used: 101.00
average falling flow: 65535.00
average rising flow: 65535.00
total water used: 51322.00
measured water used: 101.00
average falling flow: 0.00
average rising flow: 0.00


        EEPROM.commit();*/

        /// signal EEPROM write using green LED
        digitalWrite(LED_GREEN, HIGH);
        delay(20);
        digitalWrite(LED_GREEN, LOW);
        delay(20);
        digitalWrite(LED_GREEN, HIGH);
        delay(20);
      }    
    }
    /**
     * @brief Read 'total water out' and 'average flow' from non volatile EEPROM
     * 
     */
    void loadEEPROM(){
      this->totalWaterUsed = readEEprom(EEPROM_TOTAL_WATER_USED_L, EEPROM_TOTAL_WATER_USED_H);
      this->measuredWaterUsed = readEEprom(EEPROM_MEASURED_WATER_USED_L, EEPROM_MEASURED_WATER_USED_H);
      this->averageFallingFlow = readEEprom(EEPROM_FALLING_FLOW_L, EEPROM_FALLING_FLOW_H);
      this->averageRisingFlow = readEEprom(EEPROM_RISING_FLOW_L,EEPROM_RISING_FLOW_H);
      Serial.print("total water used: ");
      Serial.println(this->totalWaterUsed);
      Serial.print("measured water used: ");
      Serial.println(this->measuredWaterUsed);
      Serial.print("average falling flow: ");
      Serial.println(this->averageFallingFlow);
      Serial.print("average rising flow: ");
      Serial.println(this->averageRisingFlow);
    }
    /*double mapf(double x, double in_min, double in_max, double out_min, double out_max){
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }*/

  void update(bool verboseMode = true){

    #define minFlowOUT 20
    #define minFlowIN -20
    #define minTankFull 95
    #define maxTankEMPTY 5


    if(!levelCalcRDY && !flowCalcRDY){  /// somehow there is no new reading from sensor to process
      return;
    }

    #define SENS_BOTTOM_OFFSET 0
    #define SENS_TOP_OFFSET 0
    #define SENS_MAX_LENGTHmm 1040

    #define MIN_OPENFLOW_FALLING -20
    #define MIN_OPENFLOW_RISING 20

    #define RESERVOIR_EMPTY_LITERS 0
    #define RESERVOIR_FULL_LITERS 2000

    if(levelCalcRDY){                   /// There is a new reading from sensor for level - calculate water level
      levelCalcRDY = false;

      this->sensorSubmergedmm = getLUTmms(this->filteredFreq, true, false);
      this->submergedPercent = (this->sensorSubmergedmm + SENS_BOTTOM_OFFSET)/( SENS_BOTTOM_OFFSET + SENS_MAX_LENGTHmm + SENS_TOP_OFFSET);
      this->litersInReservoir = levelLitersFilter.getFilt(submergedPercent*RESERVOIR_FULL_LITERS);
      percent = submergedPercent*100;

    }
    if(flowCalcRDY){                    /// There is a new reading from sensor for flow - calculate flow
      flowCalcRDY = false;

      this->sensorSubmergedmm = getLUTmms(this->filteredFreq, true, false);
      this->submergedPercent = (this->sensorSubmergedmm + SENS_BOTTOM_OFFSET)/( SENS_BOTTOM_OFFSET + SENS_MAX_LENGTHmm + SENS_TOP_OFFSET);
      this->litersInReservoir = levelLitersFilter.getFilt(submergedPercent*MAX_LITERS);

      if(this->prevLitersInReservoir == -1 && this->litersInReservoir != -1){    /// if this is the first measurement, use this value as previous value
        this->prevLitersInReservoir = this->litersInReservoir;
      }

      double dLiters = this->litersInReservoir - this->prevLitersInReservoir;
      this->prevLitersInReservoir = this->litersInReservoir;

      double newFlow = (dLiters*((1000*60)/this->flowDt));
      Serial.print("dliters:");
      Serial.println((dLiters));
      Serial.print("newFlow:");
      Serial.println(newFlow);

      #define RISING_FILTERVAL 1/20    
      #define FALLING_FILTERVAL 1/10 /// 1/n average last n
      
      /// apply non uniform filtering, idea like AIMD algorithm, flow can decrease fast, but can only increase slowly 
      if(abs(newFlow) < abs(this->flow)){   /// if flow is decreasing filter less
        this->flow = 0.8 * newFlow;
        //this->flow = flowFilter.getFilt(newFlow * 0.8);
      }
      else{                                 /// if flow is increasing, filter more
        //this->flow = flowFilter.getFilt(newFlow);
        this->flow =(0.9 * this->flow) + (0.005 * newFlow); //(0.80 * this->flow) + (0.2 * newFlow) ;
      }

      
    }


    
    if(flow > MIN_OPENFLOW_FALLING && flow < MIN_OPENFLOW_RISING){            /// flow is close to zero, level is more or less static
      flowStatus = 0;                                     /// 0 - STATIC, 1 - OUT, 2 - IN
      this->ttEdge = -1;                                  /// time until tank empty/ or full is infinity
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, LOW);
      #ifdef PRINT_STATUS
      Serial.print(" Flow: Static ");
      #endif
    }
    else if(flow < MIN_OPENFLOW_FALLING){                                       /// flow is not static - water level decreasing
        this->avrgOutFlow = (this->avrgOutFlow * 0.8) + (this->flow * 0.2);     /// calculate average flow out - only if level is descending AND save to EEPROM
        this->ttEdge = abs((RESERVOIR_EMPTY_LITERS - this->litersInReservoir) / flow);              /// calculate estimated time until reservoir is empty based on current flow
        this->ttEdgeAvgOpen = abs((RESERVOIR_EMPTY_LITERS - this->litersInReservoir)/this->avrgOutFlow);/// calculate estimated time until reservoir is empty based on average flow
        flowStatus = 1;                                                         /// 0 - STATIC, 1 - OUT, 2 - IN

        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);
        #ifdef PRINT_STATUS
        Serial.print(" Flow: Out    ");
        #endif
    }
    else{                                                                   /// flow is not static - water level increasing
        this->ttEdge = abs((RESERVOIR_FULL_LITERS - this->litersInReservoir) / flow);           /// calculate estimated time until reservoir is full based on current flow
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
    /*if(levelStatus == 0 ){            /// EMPTY
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
      }*/

    //}

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
   * @brief Return string with current measurement values
   * 
   * @return String 
   */
  String getInfoStr(){
    /// TankLevel/ TankLiters/ Flow/ TMUntilEmpty/ SpentWater/ AverageFlow/ DischStarted/ Last fill time/ 
    return String(this->submergedPercent) + "/" + String(this->litersInReservoir) + "/" + "01:23:45" + "/"+ String(this->flow) + "/" + String(111) + "/" + String(222) +  "/" + String(1234567) + "/" + String(98765432)+ "/" + String(12.5) + "/" + String(filteredFreq) + "/" + "192.168.1.1" + "/" + String(this->sensorSubmergedmm);
  }/**
   * @brief Set the Total Out object
   * 
   * @param totalOut 
   */
  void setTotalOut(double totalOut){
    this->totalWaterUsed = totalOut;
    }
};
