/*
   T-Works
   Venkata Rahul, S

*/

/*
   Library URL: https://github.com/JChristensen/extEEPROM/blob/master/examples/eepromTest/eepromTest.ino
*/

#include <extEEPROM.h>
#define EEPROM_I2C_ADDR 0x50
#define EEPROM_BASE_ADDR 0x10

const uint32_t totalKBytes = 64;

extEEPROM hbad_mem(kbits_512, 1, 32, EEPROM_I2C_ADDR);

void storeParam(ctrl_parameter_t param) {
  byte dataToStore[2] = {(byte)(param.value_new_pot >> 8), (byte) (param.value_new_pot)};
  int storeAddress = EEPROM_BASE_ADDR + (4 * (param.index - 1));
  hbad_mem.write(storeAddress, dataToStore, 2);
  Serial << F("Saved")<< endl;
  Serial << F("Saved")<< endl;
  param.value_curr_mem = param.value_new_pot;
}

void retrieveParam(ctrl_parameter_t param) {
  int storeAddress = EEPROM_BASE_ADDR + 4 * (param.index - 1);
  uint8_t retrievedData[2];
  hbad_mem.read(storeAddress, retrievedData, 2);
  param.value_new_pot = (int)(retrievedData[0] << 8) + (int) retrievedData[1];
}