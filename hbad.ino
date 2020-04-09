static const uint8_t ctrl_pins[] = {A0, A1, A2, A3, A4, A5, A6};

#include <Encoder.h>
#include <Streaming.h>
#include <Wire.h>
#include "pinout.h"
#include "ctrl_display.h"
#include "lcd.h"
#include "hbad_memory.h"

volatile int currPos = 1;
int ctrlParamChangeInit = 0;
volatile int switchMode = 0;
volatile static short announced = 0;
volatile short actionPending = 1;

void setup() {
  pinMode(DISP_ENC_CLK, INPUT);
  pinMode(DISP_ENC_DT, INPUT);
  pinMode(DISP_ENC_SW, INPUT_PULLUP);
  lcd.begin(LCD_LENGTH_CHAR, LCD_HEIGHT_CHAR);
  Wire.setClock(4000000L);
  Wire.begin();
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(DISP_ENC_SW), isr_processStartEdit, HIGH);
}

void loop() {
  announce();
  processRotation();
  showSelectedParam();
  saveSelectedParam();
  if (actionPending == 1) {
    delay(2000);
  }
}

void announce() {
  if (announced == 0) {
    listDisplayMode();
    editParameterMode();
  }
}

void listDisplayMode() {
  if (switchMode == DISPLAY_MODE) {
    unsigned int nextLine = 1;
    for (int i = 0; i < MAX_CTRL_PARAMS; i++) {
      if (i % 3 == 0) {
        lcd.setCursor(0, 0);
        lcd.print(mode_headers[switchMode]);
        nextLine = 1;
      }
      cleanRow(nextLine);
      lcd.setCursor(0, nextLine);
      lcd.print(params[i].parm_name);
      lcd.setCursor(14, nextLine);
      lcd.print(params[i].value_curr_mem);
      nextLine++;
      if (nextLine > 3) {
        delay(3000);
      }
    }
  }
}

void editParameterMode() {
  if (switchMode == EDIT_MODE_ON) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(mode_headers[switchMode]);
  }
}
int processRotation() {
  int lastCLK = digitalRead(DISP_ENC_CLK);
  int cursorIndex = 0;
  /*Index_select loop*/
  while (switchMode == EDIT_MODE_ON) {
    if (lastCLK == digitalRead(DISP_ENC_CLK)) {
      continue;
    }
    if (lastCLK == 0 ) {
      continue;
    }
    if (digitalRead(DISP_ENC_CLK) == digitalRead(DISP_ENC_DT)) {
      cursorIndex--;
      if (cursorIndex <= 0) {
        cursorIndex = MAX_CTRL_PARAMS;
      }
    } else if (digitalRead(DISP_ENC_CLK) != digitalRead(DISP_ENC_DT)) {
      cursorIndex++;
      if (cursorIndex > MAX_CTRL_PARAMS) {
        cursorIndex = 1;
      }
    }
    currPos = cursorIndex;
    cleanRow(1);
    lcd.setCursor(0, 1);
    lcd.print(params[currPos - 1].parm_name);
    delay(100);
  }
  return 0;
}

void showSelectedParam() {
  while (switchMode == PAR_SELECTED_MODE) {
    lcd.setCursor(0, 1);
    lcd.print(params[currPos - 1].parm_name);
    params[currPos - 1].value_new_pot = analogRead(analog_pins[currPos - 1]);
    lcd.setCursor(10, 1);
    sprintf(paddedValue, "%4d", params[currPos - 1].value_new_pot);
    lcd.print(paddedValue);
    float actualValue = getActualParamValue(params[currPos - 1].value_new_pot, currPos - 1);
    lcd.setCursor(10, 2);
    lcd.print(actualValue,2);
    lcd.setCursor(10, 3);
    float storedValue = getActualParamValue(params[currPos - 1].value_curr_mem, currPos - 1);
    lcd.print(storedValue);
    delay(mode_loop_delays[switchMode]);
  }
}

void saveSelectedParam() {
  if (switchMode == PAR_SAVE_MODE) {
    Serial.print("Saving..");
    storeParam(params[currPos - 1]);
    lcd.setCursor(0, 4);
    lcd.print(params[currPos - 1].parm_name);
    /*
      int address = sizeof(params[currPos - 1].value_curr_mem) * currPos;
       Though the above code is generic, for simplicity of the code below,
       We are assuming four hytes here.
       https://learn.sparkfun.com/tutorials/reading-and-writing-serial-eeproms/all
    */
    /*
           Wire.beginTransmission(EEPROM_I2C_ADDR);
      int currentMemBegin = EEPROM_BASE_ADDR + (currPos - 1) * 4;
      Wire.write(currentMemBegin & 0xFF);
      Wire.write(currentMemBegin >> 8);
      Wire.write(params[currPos - 1].value_new_pot);
        Serial.print("Actual value:\t");
        Serial.println(params[currPos - 1].value_new_pot);
        Serial.print("in up:\t");
        Serial.println(params[currPos - 1].value_new_pot >> 8);
        Serial.print("in low:\t");
        Serial.println(params[currPos - 1].value_new_pot & 0xFF);
    */
//    unsigned short saveFlag = Wire.endTransmission();
      int saveFlag = 0;
    if (saveFlag == 0) {
      lcd.setCursor(14, 3);
      lcd.print(" saved");
    } else {
      lcd.setCursor(14, 3);
      lcd.print(" Error!!");
    }
    delay(500);
    switchMode = DISPLAY_MODE;
    actionPending = 0;
  }
}

int getParamValFrmMem(int paramIdx) {
  int currentMemBegin = EEPROM_BASE_ADDR + paramIdx * 4;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(currentMemBegin & 0xFF);
  Wire.write(currentMemBegin >> 8);
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDR, 2);
  delay(10);
  byte memByte = Wire.read();
  Serial.print("UB:\t");
  Serial.println(memByte);
  int sensorReading = memByte << 8;
  memByte = Wire.read();
  Serial.print("LB:\t");
  Serial.println(memByte);
  return sensorReading;
}

void isr_processStartEdit() {
  static unsigned long lastSwitchTime = 0;
  unsigned long switchTime = millis();
  if ((switchTime - lastSwitchTime) < DBNC_INTVL_SW) {
    return;
  }
  switchMode = (switchMode + 1) % 4;
  Serial.println(switchMode);
  announced = 0;
  actionPending = 1;
  lastSwitchTime = switchTime;
}

float getActualParamValue(int potValue, int paramIndex){
  float convVal = map(potValue, 0,POT_HIGH, param_range_min[paramIndex], param_range_max[paramIndex]);
//  int quot = convVal / param_incr[paramIndex];
//  return param_incr[paramIndex] * quot;
  return ((int)(convVal / param_incr[paramIndex]) + 1) * param_incr[paramIndex];
//  Serial.print("Conv value\t");
//  Serial.println(convVal);
//  Serial.print("floored\t");
//  Serial.println(retVal);
}
