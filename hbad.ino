#include <Wire.h>
#include "pinout.h"
#include "ctrl_display.h"
#include "lcd.h"
#include "hbad_serial.h"
#include "calib_calc_m_c.h"
#include "sensor_params.h"
#include "sensor_read.h"
#include "Diagnostics.h"
#include <MsTimer2.h>

volatile short currPos = 1;
unsigned short newIER = 1;
unsigned short newPeep = 5;

int ctrlParamChangeInit = 0;
volatile int switchMode = 0;
volatile static short announced = 0;
volatile boolean actionPending = false;
int lastCLK = 0;
boolean currPosChanged = 0;
#define ROT_ENC_FOR_IER (currPos == inex_rati.index)
#define ROT_ENC_FOR_PEEP (currPos == peep_pres.index)

#define READ_FROM_ENCODER ((switchMode == PAR_SELECTED_MODE) \
                           && (currPos >=0 && (currPos < MAX_CTRL_PARAMS) \
                               && (params[currPos].readPortNum == DISP_ENC_CLK)))


void setup() {
  pinMode(DISP_ENC_CLK, INPUT);
  pinMode(DISP_ENC_DT, INPUT);
  pinMode(DISP_ENC_SW, INPUT_PULLUP);
  pinMode(ADS115_INT_PIN, INPUT_PULLUP);
  lcd.begin(LCD_LENGTH_CHAR, LCD_HEIGHT_CHAR);
  Wire.setClock(4000000L);
  Wire.begin();
  hbad_mem.begin();
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(DISP_ENC_SW), isr_processStartEdit, HIGH);
  getAllParamsFromMem();
  setup_calib_calc_m_c();
  MsTimer2::set(1000, saveSensorData);
  MsTimer2::start();
  
  if(digitalRead(DISP_ENC_SW))
  {
    Diagnostics_Mode();
  }
}


void loop() {
  //  sendCommands();
  announce();
  //  Serial.println("currPos before\t");
  //  Serial.println(currPos);
  processRotation();
  //  Serial.println("currPos after\t");
  //  Serial.println(currPos);
  showSelectedParam();
  saveSelectedParam();
  if (!actionPending) {
    delay(1000);
  }
}
void sendCommands() {
  String oprName = "P";
  String command;
  char paddedValue[3];
  for (int i = 0; i < MAX_CTRL_PARAMS - 2; i++) {
    // padding(params[i].value_curr_mem, 4 );
    command = START_DELIM;
    command += VENT_MAST;
    command += oprName + i;
    sprintf(paddedValue, "%04d",
            params[i].value_curr_mem);
    command += paddedValue;
    command += END_DELIM;
    Serial.println(command);
    delay(3000);
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
    lcd.setCursor(0, 0);
    lcd.print(mode_headers[switchMode]);
    unsigned int nextLine = 1;
    for (int i = 0; i < MAX_CTRL_PARAMS; i += 2) {
      cleanRow(nextLine);
      lcd.setCursor(NAME1_DISPLAY_POS, nextLine);
      lcd.print(params[i].parm_name);
      lcd.setCursor(VALUE1_DISPLAY_POS, nextLine);
      bool doNotSkipCalib = true;
      if (i == inex_rati.index || i == peep_pres.index) {
        doNotSkipCalib = false;
      }
      if (doNotSkipCalib) {
        lcd.print(getCalibratedParam(params[i]));
      } else {
        if (i == inex_rati.index) {
          lcd.print("1:");
        }
        lcd.print(params[i].value_curr_mem);
      }
      lcd.print(" ");
      lcd.print(PAR_SEP_CHAR);
      lcd.setCursor(NAME2_DISPLAY_POS, nextLine);
      lcd.print(params[i + 1].parm_name);
      lcd.setCursor(VALUE2_DISPLAY_POS, nextLine);
      if (doNotSkipCalib) {
        lcd.print(getCalibratedParam(params[i + 1]));
      } else {
        if ((i + 1) == inex_rati.index) {
          lcd.print("1:");
        }
        lcd.print(params[i + 1].value_curr_mem);
      }
      nextLine++;
    }
  }
}

void editParameterMode() {
  if (switchMode == EDIT_MODE) {
    lcd.setCursor(0, 0);
    lcd.print(mode_headers[switchMode]);
    cleanColRow(2, 1);
    for (int i = 2; i < LCD_HEIGHT_CHAR; i++) {
      cleanRow(i);
    }
    lcd.setCursor(NAME1_DISPLAY_POS, 1);
    lcd.print(params[currPos].parm_name);
    if (currPos == inex_rati.index) {
      lcd.print(" 1:");
      lcd.setCursor(VALUE1_DISPLAY_POS + 2, 1);
    } else {
      lcd.setCursor(VALUE1_DISPLAY_POS, 1);
    }
    lcd.print(params[currPos].value_curr_mem);
  }
}
int processRotation() {
  /*Index_select loop*/
  int cursorIndex = 0;
  unsigned long lastRotateTime = 0;
  unsigned long rotateTime = millis();
  unsigned short int retVal;
  int currentStateCLK = digitalRead(DISP_ENC_CLK);
  while (switchMode == EDIT_MODE || READ_FROM_ENCODER) {
    if ((rotateTime - lastRotateTime) < DBNC_INTVL_ROT) {
      return 0;
    }
    rotateTime = millis();
    currentStateCLK = digitalRead(DISP_ENC_CLK);
    if (currentStateCLK != lastCLK && currentStateCLK == 1) {
      if (currentStateCLK != digitalRead(DISP_ENC_DT)) {
        cursorIndex++;
      } else  {
        cursorIndex--;
      }
      Serial.print("currPos\t");
      Serial.print(params[currPos].parm_name);
      Serial.println(currPos);
      if (READ_FROM_ENCODER) {
        if (ROT_ENC_FOR_IER) {
          newIER = rectifyBoundaries(newIER + cursorIndex, inex_rati.min_val, inex_rati.max_val);
          cleanRow(1);
          lcd.setCursor(VALUE1_DISPLAY_POS, 1);
          lcd.print("1:");
          lcd.print(newIER);
          retVal = newIER;
          params[inex_rati.index].value_new_pot = newIER;
        } else if (ROT_ENC_FOR_PEEP) {
          newPeep = rectifyBoundaries(newPeep + cursorIndex * peep_pres.incr, peep_pres.min_val, peep_pres.max_val);
          params[peep_pres.index].value_new_pot = newPeep;
          lcd.setCursor(VALUE1_DISPLAY_POS, 1);
          lcd.print(newPeep);
          retVal = newPeep;
        }
      } else {
        currPos = rectifyBoundaries(currPos + cursorIndex, 0, MAX_CTRL_PARAMS - 1);
      }
    }
    retVal = currPos;
    lastCLK = currentStateCLK;
    lastRotateTime = rotateTime;
  }
  return retVal;
}

void showSelectedParam() {
  while (switchMode == PAR_SELECTED_MODE) {
    actionPending = true;
    lcd.setCursor(NAME1_DISPLAY_POS, 1);
    lcd.print(params[currPos].parm_name);
    if (currPos == inex_rati.index || currPos == peep_pres.index) {
      return;
    }

    params[currPos].value_new_pot = analogRead(params[currPos].readPortNum);
    lcd.setCursor(VALUE1_DISPLAY_POS, 1);
    //    sprintf(paddedValue, "%4d", params[currPos].value_new_pot);
    //    lcd.print(paddedValue);
    //    Serial.print("pot:");Serial.print(params[currPos].readPortNum);
    //    Serial.print("\tcurrPos:");Serial.println(currPos);
    //    Serial.print("\tpot:");Serial.println(params[currPos].value_new_pot);
    float actualValue = getCalibValue(params[currPos].value_new_pot, currPos);
    lcd.setCursor(VALUE1_DISPLAY_POS, 1);
    lcd.print(actualValue, 2);
    lcd.setCursor(VALUE1_DISPLAY_POS, 2);
    float storedValue = getCalibValue(params[currPos].value_curr_mem, currPos);
    lcd.print(storedValue);
    delay(mode_loop_delays[switchMode]);
  }
}

void saveSelectedParam() {
  if (switchMode == PAR_SAVE_MODE) {
    //    Serial.println(params[currPos].value_new_pot);
    storeParam(params[currPos]);
    params[currPos].value_curr_mem = params[currPos].value_new_pot;
    //    Serial.println(params[currPos].value_curr_mem);
    switchMode = DISPLAY_MODE;
    actionPending = false;
    lcd.setCursor(0, 3);
    lcd.print(params[currPos].parm_name);
    lcd.print(" saved.....");
    delay(1000);
    cleanRow(3);
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
   The getCalibValue is for calibrating any integer to a parameter.
   The parameter is input with the help of an index
*/
int getCalibValue(int potValue, int paramIndex) {
  float convVal = map(potValue, 0, POT_HIGH, params[paramIndex].min_val, params[paramIndex].max_val);
  return ((int)(convVal / params[paramIndex].incr) + 1) * params[paramIndex].incr;
}

/*
   The below method is for a specific parameter
*/
int getCalibratedParam(ctrl_parameter_t param) {
  float convVal = map(param.value_curr_mem, 0, POT_HIGH, param.min_val, param.max_val);
  return ((int)(convVal / param.incr) + 1) * param.incr;
}

int rectifyBoundaries(int value, int minimum, int maximum) {
  if (value < minimum) {
    return maximum;
  }
  if (value > maximum) {
    return minimum;
  }
  return value;
}
