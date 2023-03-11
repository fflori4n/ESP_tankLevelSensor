void writeToEEprom(uint16_t value, uint8_t addrLow, uint8_t addrHigh, int noUpdateHist= 0){
  
  uint16_t eePromValue = (EEPROM.read(addrHigh) << 8) | (EEPROM.read(addrLow) & 0xff);
  if(abs(value - eePromValue) > noUpdateHist){
    EEPROM.write(addrLow, ((uint16_t)value >> 0) & 0xFF);
    EEPROM.write(addrHigh, ((uint16_t)value >> 8) & 0xFF);
    EEPROM.commit();
  }
}

double readEEprom(uint8_t addrLow, uint8_t addrHigh){
  return (uint16_t)(EEPROM.read(addrHigh) << 8) | (EEPROM.read(addrLow) & 0xff);
}
