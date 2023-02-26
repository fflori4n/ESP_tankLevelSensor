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
      if(disp2 == 1){
        out |= 0b10000000;
      }
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, (9 - address));  /// Byte 0 - index to change - display 0
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, out);     /// Byte 1 - new character - display 0

      out = this->to7SegmentChar(char1);
      if(disp1 == 1){
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
        writeChar((j + 1), char1, char2,0,0);
      }
    }
    void setDecPoint(char* str, byte index) {
      str[index] = 1;
    }
    void printMenu(byte page, unsigned int percent, int flow, int secsTTedge, int lvlStatus, int freq, int batv) {
      char char1, char2;
      char printBuff[] = "                ";
      char decBuff[] = "                ";

      switch (page) {
        case 0: /// print status
          {
            char lvl3 = ' ', lvl2 = ' ', lvl1 = ' ', lvl0 = ' ';
            char flow2 = ' ', flow1 = ' ', flow0 = ' ', flowSgn = ' ';
            char levelStatusChar = ' ';
            char ttmin1 = ' ', ttmin0 = ' ', ttsec1 = ' ', ttsec0 = ' ';
            char ttAverageSgn = 'A';

            percent = constrain((percent), 0, 1000);
            lvl0 = '0' + (percent % 10);
            lvl1 = '0' + (percent % 100) / 10;
            if (percent > 99) {
              lvl2 = '0' + (percent % 1000) / 100;
            }
            if (percent > 999) {
              lvl3 = '0' + (percent % 10000) / 1000;
            }

            levelStatusChar = '~';
            if (abs(flow) >= 50) {
              if (flow > 0) {
                levelStatusChar = '<';
              }
              else {
                levelStatusChar = '>';
              }
            }
            if (secsTTedge / 10000 == 1) {
              ttAverageSgn = 'C';
              if (flow < 0) {
                ttAverageSgn = 'F';
              }
            }


            flow = constrain(flow, -999, 999);
            if (flow < 0) {
              flowSgn = '-';
            }
            flow = abs(flow);
            flow0 = '0' + (flow % 10);
            if (flow > 10) {
              flow1 = '0' + ((flow % 100) / 10);
            }
            if (flow > 100) {
              flow2 = '0' + ((flow % 1000) / 100);
            }

            ttsec0 = '0' + (secsTTedge % 10); //((secsTTedge % 60) % 10);
            ttsec1 = '0' + (secsTTedge % 100) / 10; //((secsTTedge % 60) / 10);
            ttmin0 = '0' + (secsTTedge % 1000) / 100; //((secsTTedge / 60) % 10);
            ttmin1 = '0' + (secsTTedge % 10000) / 1000; //constrain(((secsTTedge / 60) / 10), 0, 9);


            snprintf(printBuff, 17, "%c%c%c%c%c%c%c%c%c%c %c%c %c%c", flowSgn, flow2, flow1, flow0, lvl3, lvl2, lvl1, lvl0, levelStatusChar, ttAverageSgn, ttmin1, ttmin0, ttsec1, ttsec0);
            setDecPoint(decBuff, 6);
          }
          break;
        case 1:
          snprintf(printBuff, 17, "FREQ    %d%d%d%d%d%d%d%d", (freq % 100000000) / 10000000, (freq % 10000000) / 1000000, (freq % 1000000) / 100000, (freq % 100000) / 10000, (freq % 10000) / 1000, (freq % 1000) / 100, (freq % 100) / 10, (freq % 10));
          break;
        case 2:
          snprintf(printBuff, 17, "BAt     %d%d%d%d  ", ((batv % 10000) / 1000), ((batv % 1000) / 100), ((batv % 100) / 10), (batv % 10));
          setDecPoint(decBuff, 9);
          break;
          /*case 3:
            snprintf(printBuff, 17, "FiLL SEC 1 45", 1,2,3,7);
            //setDecPoint(printBuff, 9);
            break;*/
      }

      digitalWrite(MAX7219_CS, HIGH);
      for (int j = 0; j < 8; j++) {

        char1 = printBuff[j];
        char2 = printBuff[j + 8];
        writeChar((j + 1), char1, char2, decBuff[j], decBuff[j+8]);
      }
    }

};
