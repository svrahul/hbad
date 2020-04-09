static const uint8_t ctrl_pins[] = {A0, A1, A2, A3, A4, A5, A6};

#include <Encoder.h>
#include <Wire.h>
#include "pinout.h"
#include "ctrl_display.h"
#include "lcd.h"
#include "hbad_serial.h"
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
  hbad_mem.begin();
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
      lcd.print(getCalibratedParam(params[i]));
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
    float actualValue = getCalibValue(params[currPos - 1].value_new_pot, currPos - 1);
    lcd.setCursor(10, 2);
    lcd.print(actualValue,2);
    lcd.setCursor(10, 3);
    float storedValue = getCalibValue(params[currPos - 1].value_curr_mem, currPos - 1);
    lcd.print(storedValue);
    delay(mode_loop_delays[switchMode]);
  }
}

void saveSelectedParam() {
  if (switchMode == PAR_SAVE_MODE) {
    storeParam(params[currPos - 1]);
    Serial.print("abcdef ");
    Serial.println(switchMode);
    lcd.setCursor(0, 4);
    lcd.print(params[currPos - 1].parm_name);
    switchMode = DISPLAY_MODE;
    actionPending = 0;
  }
}

void isr_processStartEdit() {
  static unsigned long lastSwitchTime = 0;
  unsigned long switchTime = millis();
  if ((switchTime - lastSwitchTime) < DBNC_INTVL_SW) {
    return;
  }
  switchMode = (switchMode + 1) % 4;
  announced = 0;
  actionPending = 1;
  lastSwitchTime = switchTime;
}
/*
 * The getCalibValue is for calibrating any integer to a parameter.
 * The parameter is input with the help of an index
 */
float getCalibValue(int potValue, int paramIndex){
  float convVal = map(potValue, 0,POT_HIGH, param_range_min[paramIndex], param_range_max[paramIndex]);
  return ((int)(convVal / param_incr[paramIndex]) + 1) * param_incr[paramIndex];
}

/*
 * The below method is for a specific parameter
 */
float getCalibratedParam(ctrl_parameter_t param){
  unsigned short paramIdx = param.index - 1;
  float convVal = map(param.value_curr_mem, 0,POT_HIGH, param.min_val, param.max_val);
  return ((int)(convVal / param_incr[paramIdx]) + 1) * param_incr[paramIdx];
}
