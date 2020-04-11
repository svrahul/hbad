/*
   T-Works
   Venkata Rahul, S

*/

/*
   Library URL: https://github.com/JChristensen/extEEPROM/blob/master/examples/eepromTest/eepromTest.ino
*/

#include <extEEPROM.h>

#define EEPROM_DEFAULT_DATA_WRITE_ADDR 0xA //10

#define EEPROM_O2_CALIB_ADDR 0xC //12 - 22
#define NUM_OF_SAMPLES_O2 5

#define EEPROM_PG1_CALIB_ADDR 0x28 //40 - 78
#define NUM_OF_SAMPLES_PS1 19

#define EEPROM_PS2_CALIB_ADDR  0
#define NUM_OF_SAMPLES_PS2 0

#define EEPROM_DPS1_CALIB_ADDR  0
#define NUM_OF_SAMPLES_DPS1 0

#define EEPROM_DPS2_CALIB_ADDR  0
#define NUM_OF_SAMPLES_DPS2 0

#define EEPROM_I2C_ADDR 0x50 //80
#define EEPROM_BASE_ADDR 0x64

//const uint32_t totalKBytes = 64;


extEEPROM hbad_mem(kbits_256, 1, 32, EEPROM_I2C_ADDR);

void storeParam(ctrl_parameter_t param) {
  Serial.println("Saving");
  byte dataToStore[2] = {param.value_new_pot >> 8, param.value_new_pot};
  int storeAddress = EEPROM_BASE_ADDR + (2 * (param.index));
  hbad_mem.write(storeAddress, dataToStore, 2);
  Serial.print("Stored");
  Serial.println(param.value_curr_mem);
}

void retrieveParam(ctrl_parameter_t param) {
  int storeAddress = EEPROM_BASE_ADDR + (2 * (param.index));
  byte retrievedData[2];
  hbad_mem.read(storeAddress, retrievedData, 2);
  params[param.index].value_curr_mem = (int)(retrievedData[0] << 8) + (int) retrievedData[1];
  Serial.print("Got");
  Serial.println(param.value_curr_mem);
}

void getAllParamsFromMem() {
  for (int i = 0; i < MAX_CTRL_PARAMS; i++) {
    retrieveParam(params[i]);
    Serial.print("Mem\t");
    Serial.print(i);
    Serial.print("\t");
    Serial.println(params[i].value_curr_mem);
  }
}
  
  
void storeCalibParam(int storeAddress, int data) {
  Serial.println("Saving calib");
  byte dataToStore[2] = {data >> 8, data};
  hbad_mem.write(storeAddress, dataToStore, 2);
}

int retrieveCalibParam(int address) {
  byte retrievedData[2];
  hbad_mem.read(address, retrievedData, 2);
  return ((int)(retrievedData[0] << 8) + (int) retrievedData[1]);
}
