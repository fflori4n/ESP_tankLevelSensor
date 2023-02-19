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
            /*msToStart-= 10;
            if(msToStart%100 == 0){
              Serial.println(msToStart/100);
            }
            if(msToStart<= 0){*/
              Serial.print("0");
              Serial.print(",");
              Serial.println((int)this->freq);
              calibrationStep++;
              tValveOpened = esp_timer_get_time();
            /*}*/
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
      double correctedOut = 0;
      
      double LUT[] = {
15330  , 1000.96 ,
15360 , 997.28  ,
15380 , 993.6 ,
15410 , 989.92  ,
15490 , 986.24  ,
15510 , 982.56  ,
15540 , 978.88  ,
15580 , 975.2 ,
15620 , 971.52  ,
15640 , 967.84  ,
15700 , 964.16  ,
15750 , 960.48  ,
15910 , 956.8 ,
15950 , 953.12  ,
15980 , 949.44  ,
16020 , 945.76  ,
16050 , 942.08  ,
16070 , 938.4 ,
16120 , 934.72  ,
16150 , 931.04  ,
16190 , 927.36  ,
16230 , 923.68  ,
16290 , 920 ,
16330 , 916.32  ,
16410 , 912.64  ,
16470 , 908.96  ,
16560 , 905.28  ,
16600 , 901.6 ,
16660 , 897.92  ,
16700 , 894.24  ,
16740 , 890.56  ,
16770 , 886.88  ,
16810 , 883.2 ,
16860 , 879.52  ,
16920 , 875.84  ,
16950 , 872.16  ,
17000 , 868.48  ,
17050 , 864.8 ,
17090 , 861.12  ,
17190 , 857.44  ,
17350 , 853.76  ,
17390 , 850.08  ,
17460 , 846.4 ,
17520 , 842.72  ,
17550 , 839.04  ,
17580 , 835.36  ,
17620 , 831.68  ,
17640 , 828 ,
17670 , 824.32  ,
17720 , 820.64  ,
17770 , 816.96  ,
17880 , 813.28  ,
17950 , 809.6 ,
18000 , 805.92  ,
18140 , 802.24  ,
18180 , 798.56  ,
18220 , 794.88  ,
18260 , 791.2 ,
18290 , 787.52  ,
18350 , 783.84  ,
18390 , 780.16  ,
18430 , 776.48  ,
18480 , 772.8 ,
18580 , 769.12  ,
18630 , 765.44  ,
18710 , 761.76  ,
18770 , 758.08  ,
18840 , 754.4 ,
19020 , 750.72  ,
19120 , 747.04  ,
19160 , 743.36  ,
19180 , 739.68  ,
19280 , 736 ,
19330 , 732.32  ,
19410 , 728.64  ,
19460 , 724.96  ,
19520 , 721.28  ,
19580 , 717.6 ,
19620 , 713.92  ,
19670 , 710.24  ,
19770 , 706.56  ,
19820 , 702.88  ,
19970 , 699.2 ,
20070 , 695.52  ,
20150 , 691.84  ,
20210 , 688.16  ,
20310 , 684.48  ,
20380 , 680.8 ,
20430 , 677.12  ,
20480 , 673.44  ,
20530 , 669.76  ,
20580 , 666.08  ,
20680 , 662.4 ,
20760 , 658.72  ,
20850 , 655.04  ,
21000 , 651.36  ,
21120 , 647.68  ,
21200 , 644 ,
21260 , 640.32  ,
21310 , 636.64  ,
21380 , 632.96  ,
21480 , 629.28  ,
21630 , 625.6 ,
21690 , 621.92  ,
21750 , 618.24  ,
21820 , 614.56  ,
21860 , 610.88  ,
21990 , 607.2 ,
22110 , 603.52  ,
22290 , 599.84  ,
22400 , 596.16  ,
22470 , 592.48  ,
22530 , 588.8 ,
22590 , 585.12  ,
22670 , 581.44  ,
22780 , 577.76  ,
22870 , 574.08  ,
22950 , 570.4 ,
23030 , 566.72  ,
23130 , 563.04  ,
23290 , 559.36  ,
23370 , 555.68  ,
23490 , 552 ,
23770 , 548.32  ,
23910 , 544.64  ,
23990 , 540.96  ,
24080 , 537.28  ,
24170 , 533.6 ,
24240 , 529.92  ,
24410 , 526.24  ,
24490 , 522.56  ,
24560 , 518.88  ,
24650 , 515.2 ,
24710 , 511.52  ,
24840 , 507.84  ,
25020 , 504.16  ,
25140 , 500.48  ,
25270 , 496.8 ,
25510 , 493.12  ,
25590 , 489.44  ,
25660 , 485.76  ,
25740 , 482.08  ,
25900 , 478.4 ,
26050 , 474.72  ,
26140 , 471.04  ,
26240 , 467.36  ,
26410 , 463.68  ,
26510 , 460 ,
26620 , 456.32  ,
26820 , 452.64  ,
26980 , 448.96  ,
27230 , 445.28  ,
27570 , 441.6 ,
27700 , 437.92  ,
27800 , 434.24  ,
27940 , 430.56  ,
28080 , 426.88  ,
28250 , 423.2 ,
28380 , 419.52  ,
28480 , 415.84  ,
28630 , 412.16  ,
28790 , 408.48  ,
28920 , 404.8 ,
29120 , 401.12  ,
29300 , 397.44  ,
29540 , 393.76  ,
29730 , 390.08  ,
29940 , 386.4 ,
30060 , 382.72  ,
30270 , 379.04  ,
30420 , 375.36  ,
30630 , 371.68  ,
30820 , 368 ,
30980 , 364.32  ,
31130 , 360.64  ,
31270 , 356.96  ,
31440 , 353.28  ,
31730 , 349.6 ,
32220 , 345.92  ,
32520 , 342.24  ,
32730 , 338.56  ,
33100 , 334.88  ,
33270 , 331.2 ,
33480 , 327.52  ,
33700 , 323.84  ,
33910 , 320.16  ,
34140 , 316.48  ,
34310 , 312.8 ,
34550 , 309.12  ,
34850 , 305.44  ,
35070 , 301.76  ,
35290 , 298.08  ,
35750 , 294.4 ,
36100 , 290.72  ,
36280 , 287.04  ,
36480 , 283.36  ,
36670 , 279.68  ,
36860 , 276 ,
37020 , 272.32  ,
37180 , 268.64  ,
37640 , 264.96  ,
38040 , 261.28  ,
38250 , 257.6 ,
38590 , 253.92  ,
38970 , 250.24  ,
39440 , 246.56  ,
40730 , 242.88  ,
41040 , 239.2 ,
41340 , 235.52  ,
41720 , 231.84  ,
42030 , 228.16  ,
42320 , 224.48  ,
42670 , 220.8 ,
42970 , 217.12  ,
43330 , 213.44  ,
43660 , 209.76  ,
44000 , 206.08  ,
44450 , 202.4 ,
45240 , 198.72  ,
45630 , 195.04  ,
46050 , 191.36  ,
46450 , 187.68  ,
46860 , 184 ,
47270 , 180.32  ,
47840 , 176.64  ,
48260 , 172.96  ,
48740 , 169.28  ,
49120 , 165.6 ,
49420 , 161.92  ,
49910 , 158.24  ,
50330 , 154.56  ,
51110 , 150.88  ,
53120 , 147.2 ,
54470 , 143.52  ,
55030 , 139.84  ,
55590 , 136.16  ,
56210 , 132.48  ,
56880 , 128.8 ,
57570 , 125.12  ,
58360 , 121.44  ,
59070 , 117.76  ,
59700 , 114.08  ,
60370 , 110.4 ,
61170 , 106.72  ,
61970 , 103.04  ,
63200 , 99.36 ,
64170 , 95.68 ,
65050 , 92  ,
65930 , 88.32 ,
67140 , 84.64 ,
68480 , 80.96 ,
69440 , 77.28 ,
70400 , 73.6  ,
71250 , 69.92 ,
72220 , 66.24 ,
73310 , 62.56 ,
74970 , 58.88 ,
77210 , 55.2  ,
83640 , 51.52 ,
87970 , 47.84 ,
90080 , 44.16 ,
92410 , 40.48 ,
93490 , 36.8  ,
95090 , 33.12 ,
95600 , 29.44 ,
95690 , 25.76 ,
95720 , 22.08 ,
95970 , 18.4  ,
96060 , 14.72 ,
96160 , 11.04 ,
96230 , 7.36  ,
96300 , 3.68,  
96320 , 0
      };
      double LUTCorrection[] = {
3  , 0 ,
52  , 50  ,
93  , 100 ,
134 , 150 ,
166 , 200 ,
212 , 250 ,
247 , 300 ,
295 , 350 ,
342 , 400 ,
380 , 450 ,
423 , 500 ,
462 , 550 ,
510 , 600 ,
558 , 650 ,
607 , 700 ,
655 , 750 ,
702 , 800 ,
752 , 850 ,
804 , 900 ,
855 , 950 ,
900 , 1000  
      };
      
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

      in = out;
      if(in > LUTCorrection[(sizeof(LUTCorrection) / sizeof(LUTCorrection[0]))-2]){ /// if measured value is bigger than bigest value in lookup table, use bigest value
        out = LUTCorrection[(sizeof(LUTCorrection) / sizeof(LUTCorrection[0]))-1];
      }
      else if(in < LUTCorrection[0]){                           /// if measured value is smaller than smallest value in lookup table, use smallest value
        out = LUTCorrection[1];
      }
      for(int i=0; i< ((sizeof(LUTCorrection) / sizeof(LUTCorrection[0]))-2); i+=2){          /// Loop over LUT, get two points next to each other
        if(in > LUTCorrection[i] && in <= LUTCorrection[i+2]){                                /// if measured value is between the two points
          double percent = ((in - LUTCorrection[i])/(LUTCorrection[i+2] - LUTCorrection[i]));           /// get the distance in % from the lower point

          out = LUTCorrection[i+1] +((LUTCorrection[i+3] - LUTCorrection[i+1]) * percent);              /// use linear approximation between points to get output
          //out = LUT[i+1];
          break;
        }
      }
  
      #ifdef PRINT_RAWFREQ
      Serial.print("Corrected to: ");
      Serial.println(out);
      #endif
      
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
