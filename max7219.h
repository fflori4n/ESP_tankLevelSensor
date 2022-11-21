#define MAX7219_CLK 12
#define MAX7219_CS 14
#define MAX7219_DIN 27

/**
 * @brief initialise display pins
 * 
 */
void disp_init(){
  digitalWrite(MAX7219_CS, HIGH);
  pinMode(MAX7219_DIN, OUTPUT);
  pinMode(MAX7219_CS, OUTPUT);
  pinMode(MAX7219_CLK, OUTPUT);
}
/**
 * @brief clear display buffers and shift out blank display characters
 * 
 */
void blankDispMem() {
#define dispLen 8
  for (byte i = 1; i < dispLen + 1; i++) {
    digitalWrite(MAX7219_CS, LOW);                            /// Chip select pulled low signals start of communication, display is waiting for 2 bytes of data
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, i);          /// i is index of character in display module
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, 0b00000000); /// 0b00000000 is one byte character code of 7segment character, 0 - led off, 1 - led on
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, i);          /// repeat once more, because we use two display modules, first one takes 2Bytes and forwards the rest to the second
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, 0b00000000);
    digitalWrite(MAX7219_CS, HIGH);                           /// end of messege
  }
}
/**
 * @brief Shifts out 'data' for first disp. module and 'data1' for second, the two characters will be placed at diplay index 'address'
 * 
 * @param address 
 * @param data 
 * @param data1 
 */
void send2Disp(byte address, byte data, byte data1){
  digitalWrite(MAX7219_CS, LOW);                          /// start transmission
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);  /// Byte 0 - index to change - display 0
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, data);     /// Byte 1 - new character - display 0
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);  /// Byte 2 - index to change - display 1
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, data1);    /// Byte 3 - new character - display 1
  digitalWrite(MAX7219_CS, HIGH);                         /// end transmission
}
/*void printByte(byte address, byte code){
  digitalWrite(MAX7219_CS, LOW);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, code);
  digitalWrite(MAX7219_CS, HIGH);
}*/
/**
 * @brief Translate 'simbol' and 'simbol1'(the two characters to be written to the display) to their 7segment representation, and write to display
 * 
 * @param address index of character
 * @param simbol to be written to display 0
 * @param simbol1 to be writtern to dispay1
 * @param point does the first display character need decimal point
 * @param point1 does the second display character need decimal point
 */
void printChar(byte address, char simbol, char simbol1, boolean point, boolean point1) {
#define dispLen 8
  
  /// Plausible input characters, and their seven segment code
  const byte simbCodes[][2] =
  {
    {'_', 0b00001000},
    {'0', 0b01111110},
    {'1', 0b00110000},
    {'2', 0b01101101},
    {'3', 0b01111001},
    {'4', 0b00110011},
    {'5', 0b01011011},
    {'6', 0b01011111},
    {'7', 0b01110000},
    {'8', 0b01111111},
    {'9', 0b01111011},
    {'.', 0b10000000},
    {' ', 0b00000000},
    {'A', 0b01110111},
    {'B', 0b00011111},
    {'C', 0b01001110},
    {'D', 0b00111101},
    {'E', 0b01001111},
    {'F', 0b01000111},
    {'L', 0b00001110},
    {'P', 0b01100111},
    {'R', 0b11100111},
    {'S', 0b01011011},
    {'T', 0b01110000},
    {'O', 0b01111110},
    {'U', 0b00111110},
    {'N', 0b01110110},
    {'-', 0b00000001},
    {'Y', 0b01000000}
  };
  address = (dispLen + 1) - address; /// set index to start from 0, like so: 76543210
  digitalWrite(MAX7219_CS, LOW);

  bool charFound = false;
  for (int i = 0; i < (sizeof (simbCodes) / sizeof (simbCodes[0])); i++) {  /// loop over 'simbCodes' array, and find seven segment code for character
    if (simbol == simbCodes[i][0]) {
      //digitalWrite(MAX7219_CS, LOW);
      charFound = true;
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);
      if (point) {
        shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, (simbCodes[i][1] | 0b10000000));
      }
      else {
        shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, simbCodes[i][1]);
      }
    }
  }
  /// CHAR NOT FOUND, SHIFT OUT _
  if(!charFound){
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, simbCodes[0][1]);
  }

  charFound = false;
  for (int i = 0; i < (sizeof (simbCodes) / sizeof (simbCodes[0])); i++) {
    if (simbol1 == simbCodes[i][0]) {
      charFound = true;
      shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);
      if (point1) {
        shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, (simbCodes[i][1] | 0b10000000));
      }
      else {
        shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, simbCodes[i][1]);
      }
    }
  }
  /// CHAR NOT FOUND, SHIFT OUT _
  if(!charFound){
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, simbCodes[0][1]);
  }
  digitalWrite(MAX7219_CS, HIGH);
}
void printText(){
  char topText[] = "ER  SENS";
  char bottomText[] = "FAULT   ";

  /*for(byte i=0; i< 9; i++){
    topText[i] = msg[i];
    bottomText[i] = msg[i];
  }*/

  blankDispMem();
  for (byte i = 1; i < 9; i++) {
      printChar(i, bottomText[i-1], topText[i-1], false, false);
  }

}
void printValues(int level, int flow, int avgSgn, int ttEdge, unsigned int flowStatus, char msgA[] = "", char msgB[] = "", char msgC[] = "") {

  char dispStr[9];
  char dispStr1[9] = "--------";
  char aStr[] = "    ";
  char bStr[] = "    ";
  char statusDig = '-';
  uint8_t A0dig, A1dig, A2dig;
  uint8_t B0dig, B1dig, B2dig, B3dig;
  uint8_t C0dig, C1dig, C2dig, C3dig;
  static uint8_t blinker = 0;

  if(flowStatus == 0){
    statusDig = '-';
  }else if(flowStatus == 1){
    statusDig = '_';
  }else{
    statusDig = 'Y';
  }

  /// TOP DISPLAY
  char minusStr = ' ';
  if (flow < 0) {
    minusStr = '-';
  }

  flow = abs(flow);
  A0dig = flow / 100;
  A1dig = (flow % 100) / 10;
  A2dig = flow % 10;

  if (strcmp(msgA,"") == 0){
    sprintf(aStr, "%c%d%d%d",minusStr, A0dig, A1dig, A2dig);
  }
  else{
    sprintf(aStr, msgA);
  }

  B0dig = level / 1000;
  B1dig = (level % 1000) / 100;
  B2dig = (level % 100) / 10;
  B3dig = (level % 10);

  if (strcmp(msgB,"") == 0){
    if(B0dig == 0 && B1dig == 0){
       sprintf(bStr, "  %d%d", B2dig, B3dig);
    }
    else if(B0dig == 0){
       sprintf(bStr, " %d%d%d", B1dig, B2dig, B3dig);
    }
    else{
       sprintf(bStr, "%d%d%d%d",B0dig, B1dig, B2dig, B3dig);
    }
  }
  else{
    sprintf(aStr, msgA);
  }
  
  /// BOTTOM DISPLAY
  if (strcmp(msgC,"") == 0){
    char avgOrCurrent = ' ';  /// current
    C0dig = ((ttEdge %10000)/ 1000);
    C1dig = (ttEdge % 1000) / 100;
    C2dig = (ttEdge % 100) / 10;
    C3dig = (ttEdge % 10);

    if(ttEdge/10000 == 0){
      avgOrCurrent = 'A';
    }
    else{
      avgOrCurrent = ' ';
    }
    sprintf(dispStr, "%c%c %d%d %d%d", statusDig, avgOrCurrent, C0dig, C1dig, C2dig, C3dig);
  }
  else{
  sprintf(dispStr, msgC);
  }

  sprintf(dispStr1, "%s%s", aStr, bStr);
  
  /// PRINT STRINGS TO DISPLAYS
  blankDispMem();
  for (byte i = 1; i < 9; i++) {
    if (i == 7 && dispStr[i - 1] != '-' && dispStr[i - 1] != ' ') {
      printChar(i, dispStr[i - 1], dispStr1[i - 1], false, true);
    }
    else {
      printChar(i, dispStr[i - 1], dispStr1[i - 1], false, false);
    }

  }
}
