#define MAX7219_CLK 12
#define MAX7219_CS 14
#define MAX7219_DIN 27

char simb7Seg[] = {
  '\0', 0b00000000,
  ' ', 0b00000000,
  '.', 0b10000000,
  '_', 0b00001000,
  '-', 0b00000001,
  '<', 0b00001000,
  '>', 0b01000000,
  '^', 0b00100000,
  '#', 0b00010000,
  '~', 0b00000001,
  '?', 0b11100001,
  '0', 0b01111110,
  '1', 0b00110000,
  '2', 0b01101101,
  '3', 0b01111001,
  '4', 0b00110011,
  '5', 0b01011011,
  '6', 0b01011111,
  '7', 0b01110000,
  '8', 0b01111111,
  '9', 0b01111011,
  'A', 0b01110111,
  'a', 0b01111101,
  'B', 0b00011111,
  'b', 0b00011111,
  'C', 0b01001110,
  'c', 0b00001101,
  'D', 0b00111101,
  'd', 0b00111101,
  'E', 0b01001111,
  'e', 0b01001111,
  'F', 0b01000111,
  'f', 0b01000111,
  'G', 0b01011110,
  'g', 0b01011110,
  'H', 0b00010111,
  'h', 0b00010111,
  'I', 0b00000011,
  'i', 0b00110000,
  'j', 0b01011000,
  'J', 0b01011000,
  'K', 0b01010111,
  'k', 0b01010111,
  'L', 0b00001110,
  'l', 0b00001100,
  'M', 0b01010101,
  'm', 0b01010101,
  'N', 0b00010101,
  'n', 0b00010101,
  'o', 0b00011101,
  'O', 0b01111110,
  'p', 0b01100111,
  'P', 0b01100111,
  'Q', 0b01110011,
  'q', 0b01110011,
  'R', 0b11100111,
  'r', 0b00000101,
  'S', 0b01011010,
  's', 0b01011010,
  'T', 0b01110000,
  't', 0b01000110,
  'u', 0b00011100,
  'U', 0b00011100,
  'V', 0b00101010,
  'v', 0b00101010,
  'W', 0b00111110,
  'w', 0b00111110,
  'X', 0b10100100,
  'x', 0b10100100,
  'Y', 0b00111011,
  'y', 0b00111011,
  'Z', 0b01101100,
  'z', 0b01101100
};
class Display8x8 {

#define NUMOF_8x8DISP_MODULES 2
  private:

    char getDigit(int value, int digit, bool useSpace = false){
      if(digit == -1){ /// return sign
        if(value < 0){
          return '-';
        }
        return ' ';
      }
      if(useSpace && ('0' + (abs(value)%((int)pow(10,(digit + 1))))/ ((int)pow(10, digit))) == '0'){
        if(value/(int)pow(10, digit + 1) == 0){
          return ' ';
        }
      }
      return '0' + (abs(value)%((int)pow(10,(digit + 1))))/ ((int)pow(10, digit));
    }

    void blankDispMem(byte numOfDisplays) {
#define dispLen 8
      for (byte i = 1; i < dispLen + 1; i++) {
        digitalWrite(MAX7219_CS, LOW);                              /// Chip select pulled low signals start of communication, display is waiting for 2 bytes of data
        for (int j = 0; j < numOfDisplays; j++) {                   /// send two bytes to each display
          shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, i);          /// i is index of character in display module
          shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, 0b00000000); /// 0b00000000 is one byte character code of 7segment character, 0 - led off, 1 - led on
        }
        digitalWrite(MAX7219_CS, HIGH);                             /// end of messege
      }
    }

    char to7SegmentChar(char charIn) {

      for (int i = 0; i < (sizeof(simb7Seg) / sizeof(simb7Seg[0])); i += 2) {
        if (simb7Seg[i] == charIn) {
          return simb7Seg[i + 1];
        }
      }
      return charIn;
    }

    void writeChar(byte address, char char1, char char2, int disp1, int disp2) {

      digitalWrite(MAX7219_CS, LOW);                          /// start transmission

      char out = this->to7SegmentChar(char2);
      if (disp2 == 1) {
        out |= 0b10000000;
      }
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, (9 - address));  /// Byte 0 - index to change - display 0
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, out);     /// Byte 1 - new character - display 0

      out = this->to7SegmentChar(char1);
      if (disp1 == 1) {
        out |= 0b10000000;
      }
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, (9 - address));  /// Byte 0 - index to change - display 0
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, out);     /// Byte 1 - new character - display 0

      digitalWrite(MAX7219_CS, HIGH);                         /// end transmission
    }

  public:

    Display8x8() {};
    void init() {
      digitalWrite(MAX7219_CS, HIGH);
      pinMode(MAX7219_DIN, OUTPUT);
      pinMode(MAX7219_CS, OUTPUT);
      pinMode(MAX7219_CLK, OUTPUT);

      this->blankDispMem(2);
    }

    void send2Disp(byte address, byte data, byte data1) {
      digitalWrite(MAX7219_CS, LOW);                          /// start transmission
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);  /// Byte 0 - index to change - display 0
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, data);     /// Byte 1 - new character - display 0
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);  /// Byte 2 - index to change - display 1
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, data1);    /// Byte 3 - new character - display 1
      digitalWrite(MAX7219_CS, HIGH);                         /// end transmission
    }

    void printStr(char* strToPrint) {
      char char1, char2;
      char printBuff[] = "1234123456785678";
      snprintf(printBuff, 17, "%s", strToPrint);
      //snprintf(printBuff, 17, "%s",   "BAT     FAULT   ");
      //snprintf(printBuff, 17, "%s", "SENS    FAULT   ");
      //snprintf(printBuff, 17, "%s", "CALIB           ");
      //snprintf(printBuff, 17, "%s", "ERR     NO CON  ");
      digitalWrite(MAX7219_CS, HIGH);
      for (int j = 0; j < 8; j++) {

        char1 = printBuff[j];
        char2 = printBuff[j + 8];
        writeChar((j + 1), char1, char2, 0, 0);
      }
    }
    void setDecPoint(char* str, byte index) {
      str[index] = 1;
    }
    void printConfirm(byte twists){
      char char1, char2;
      char printBuff[] = "                ";
      char decBuff[] = "                ";

      for(int i=0; i < twists; i++){
        printBuff[i] = '.';
      }
      if(twists > 8){
        snprintf(printBuff, 17, "........      OK");
      }

      digitalWrite(MAX7219_CS, HIGH);
      for (int j = 0; j < 8; j++) {

        char1 = printBuff[j];
        char2 = printBuff[j + 8];
        writeChar((j + 1), char1, char2, decBuff[j], decBuff[j + 8]);
      }
    }
    void printMenu(byte page, unsigned int percent, double flow, int secsTTedge, int lvlStatus, int freq, int batv, int minBatVoltage, int nonCorrectedLut, int correctedLut, int total, int measured, int fallingFlow, int risingFlow, int wifiEnabled) {
      char char1, char2;
      char printBuff[] = "                ";
      char decBuff[] = "                ";

      char levelStatusChar = ' ';
      char ttmin1 = ' ', ttmin0 = ' ', ttsec1 = ' ', ttsec0 = ' ';
      char ttAverageSgn = 'A';

      char * wifiEnabledStr = " ON";
      //char * wifiEnabledStr = "OFF";
      
      percent = constrain((percent), 0, 1000);

      ttAverageSgn = '#';
      levelStatusChar = '~';
      if (abs(flow) >= 20) {
        if (flow > 0) {
          levelStatusChar = '<';
          ttAverageSgn = '^';
        }
        else {
          levelStatusChar = '>';
          ttAverageSgn = '#';
        }
      }
      
     /* if (secsTTedge / 10000 == 1) {
        ttAverageSgn = 'C';
        if (flow < 0) {
          ttAverageSgn = 'F';
        }
      }*/


      flow = constrain(flow*100, -99999, 99999);

      byte tteHours = ((secsTTedge % (24*3600))/3600);
      byte tteMinutes = (secsTTedge % 3600)/60;
      byte tteSeconds = secsTTedge % 60;

      if(tteHours >= 1){
        tteMinutes = 59;
        tteSeconds = 59;
      }

     /* ttsec0 = '0' + (secsTTedge % 10); //((secsTTedge % 60) % 10);
      ttsec1 = '0' + (secsTTedge % 100) / 10; //((secsTTedge % 60) / 10);
      ttmin0 = '0' + (secsTTedge % 1000) / 100; //((secsTTedge / 60) % 10);
      ttmin1 = '0' + (secsTTedge % 10000) / 1000; //constrain(((secsTTedge / 60) / 10), 0, 9);*/

      switch (page) {
        case 0: /// print status
          {
            if((int)freq <= 0){          /// if sensor not connected, print error to display, else print sensor readings to display
              snprintf(printBuff, 17, "ERR     NO CON  ");
            }
            else if((int)freq >= 150000){
              snprintf(printBuff, 17, "SENS    FAULT   ");
            }
            else{
              snprintf(printBuff, 17, "%c%c%c%c%c%c%c%c%c%c %c%c %c%c", getDigit(flow, -1), getDigit(flow, 4, true), getDigit(flow, 3, true), getDigit(flow, 2), getDigit(percent, 3, true), getDigit(percent, 2, true), getDigit(percent, 1), getDigit(percent, 0), levelStatusChar, ttAverageSgn, getDigit(tteMinutes,1), getDigit(tteMinutes,0), getDigit(tteSeconds,1), getDigit(tteSeconds,0));
              setDecPoint(decBuff, 6);
            } 
          }
          break;
        case 1: /// view raw counts from probe
          snprintf(printBuff, 17, "BAt %c%c%c%c    %c%c%c%c", getDigit(batv,3, true), getDigit(batv,2), getDigit(batv,1), getDigit(batv,0), getDigit(minBatVoltage,3, true), getDigit(minBatVoltage,2), getDigit(minBatVoltage,1), getDigit(minBatVoltage,0));
          setDecPoint(decBuff, 5);
          setDecPoint(decBuff, 13);
          break;
        case 2: /// print battery voltage
          snprintf(printBuff, 17, "LOG     %c%c%c%c%c%c%c%c",getDigit(measured, 7, true),getDigit(measured, 6, true),getDigit(measured, 5, true),getDigit(measured, 4, true), getDigit(measured, 3, true),getDigit(measured, 2, true),getDigit(measured, 1, true),getDigit(measured, 0));
          break;
        case 3: ///
          snprintf(printBuff, 17, "totAL   %c%c%c%c%c%c%c%c",getDigit(total, 7, true),getDigit(total, 6, true),getDigit(total, 5, true),getDigit(total, 4, true), getDigit(total, 3, true),getDigit(total, 2, true),getDigit(total, 1, true),getDigit(total, 0));
          break;
        case 4:
          snprintf(printBuff, 17, "FLOLW     %c%c%c%c%c%c", getDigit(flow, -1),getDigit(flow, 4, true), getDigit(flow, 3, true), getDigit(flow, 2), getDigit(flow, 1), getDigit(flow, 0));
          setDecPoint(decBuff, 13);
          break;
        case 5:
          snprintf(printBuff, 17, "NAF       %c%c%c%c%c%c", getDigit(fallingFlow, -1),getDigit(fallingFlow, 4, true), getDigit(fallingFlow, 3, true), getDigit(fallingFlow, 2), getDigit(fallingFlow, 1), getDigit(fallingFlow, 0));
          setDecPoint(decBuff, 13);
          break;
        case 6:
          snprintf(printBuff, 17, "PAF       %c%c%c%c%c%c", getDigit(risingFlow, -1),getDigit(risingFlow, 4, true),getDigit(risingFlow, 3, true),getDigit(risingFlow, 2),getDigit(risingFlow, 1),getDigit(risingFlow, 0));
          setDecPoint(decBuff, 13);
          break;
        case 7:
          snprintf(printBuff, 17, "               .");
          break;
         case 8:
          if(wifiEnabled){
            snprintf(printBuff, 17, "LWiFi         ON");
          }
          else{
            snprintf(printBuff, 17, "LWiFi        OFF");
          }
          break;
        case 9: /// blank
          snprintf(printBuff, 17, "FREQ    %c%c%c%c%c%c%c%c", getDigit(freq, 7, true), getDigit(freq, 6, true), getDigit(freq, 5, true), getDigit(freq, 4, true), getDigit(freq, 3, true), getDigit(freq, 2, true), getDigit(freq, 1, true), getDigit(freq, 0));
          break;
        case 10:
          snprintf(printBuff, 17, "C %c%c%c%c%c%cL %c%c%c%c%c%c",getDigit(nonCorrectedLut, 5, true),getDigit(nonCorrectedLut, 4, true),getDigit(nonCorrectedLut, 3, true), getDigit(nonCorrectedLut, 2, true), getDigit(nonCorrectedLut, 1, true), getDigit(nonCorrectedLut, 0), getDigit(correctedLut, 5, true),getDigit(correctedLut, 4, true) ,getDigit(correctedLut, 3, true), getDigit(correctedLut, 2, true), getDigit(correctedLut, 1, true), getDigit(correctedLut, 0));
          break;
      }

      digitalWrite(MAX7219_CS, HIGH);
      for (int j = 0; j < 8; j++) {

        char1 = printBuff[j];
        char2 = printBuff[j + 8];
        writeChar((j + 1), char1, char2, decBuff[j], decBuff[j + 8]);
      }
    }

};
