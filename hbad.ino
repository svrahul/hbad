static const uint8_t ctrl_pins[] = {A0, A1, A2, A3, A4, A5, A6};

//#include <Encoder.h>
#include <Wire.h>
#include "pinout.h"
#include "ctrl_display.h"
#include "lcd.h"
#include "hbad_serial.h"
//#include "hbad_memory.h"
#include "calib_calc_m_c.h"

volatile short currPos = 1;
unsigned short newIER = 1;
unsigned short newPeep = 5;

int ctrlParamChangeInit = 0;
volatile int switchMode = 0;
volatile static short announced = 0;
volatile boolean actionPending = true;
boolean ierSelect = false;
boolean peepSelect = false;

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
  setup_calib_calc_m_c();
  calc_m_c_for_all_sensors();
  ADS1115_init();
}

void loop() {
  sendCommands();
  announce();
  processRotation();
  showSelectedParam();
  saveSelectedParam();
  if (actionPending) {
    delay(2000);
  }
}
void sendCommands() {
  String oprName="P";
  String command;
  char paddedValue[3];
  int size = sizeof(params)/sizeof(params[0]);
  for(int i=0;i<size-2;i++){
   // padding(params[i].value_curr_mem, 4 );
   command = START_DELIM;
   command +=VENT_MAST;
   command += oprName + i;
     sprintf(paddedValue, "%04d",
        params[i].value_curr_mem);
   command += paddedValue;
   command += END_DELIM;
   Serial.println(command);
   delay(3000);
  }
  
}
float OOM202_GetOxygenPercentage(void)
{
  float O2Percentage=0;
  float SensorVolt;
  SensorVolt=ADC_ReadVolageOnATMega2560(OXYGEN_ANALOG_PIN);
  O2Percentage = get_o2_percentage(SensorVolt);
  return(O2Percentage);
}

float PS_GetPressureValue(int Channel)
{
  float Pressure=0.0;
  float SensorVolt;
  SensorVolt = ADS1115_ReadVolageOverI2C(Channel);
  Pressure = get_pressure_value(SensorVolt, Channel);
  return(Pressure);
}

void announce() {
  if (announced == 0) {
    listDisplayMode();
    editParameterMode();
  }
}

void listDisplayMode() {
  Serial.print("switchMode\t");
  Serial.println(switchMode);
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
  while (switchMode == EDIT_MODE_ON || ierSelect || peepSelect) {
    if (lastCLK == digitalRead(DISP_ENC_CLK)) {
      continue;
    }
    if (lastCLK == 0 ) {
      continue;
    }
    lcd.setCursor(0, 1);
    lcd.print(params[currPos - 1].parm_name);
    if (digitalRead(DISP_ENC_CLK) == digitalRead(DISP_ENC_DT)) {
      cursorIndex--;
    } else if (digitalRead(DISP_ENC_CLK) != digitalRead(DISP_ENC_DT)) {
      cursorIndex++;
    }
    if (ierSelect) {
      newIER = rectifyBoundaries(newIER + cursorIndex, inex_rati.min_val, inex_rati.max_val);
      cleanRow(1);
      lcd.setCursor(10, 1);
      lcd.print("1:");
      lcd.print(newIER);
      return newIER;
    } else if (peepSelect) {
      newPeep = rectifyBoundaries(newPeep + cursorIndex * peep_pres.incr, peep_pres.min_val, peep_pres.max_val);
      lcd.setCursor(10, 1);
      lcd.print(newPeep);
      return newPeep;
    } else {
      currPos = currPos + cursorIndex;
      currPos = rectifyBoundaries(currPos + cursorIndex, 0, MAX_CTRL_PARAMS);
      cleanRow(1);
      return currPos;
    }
    delay(100);
  }
  return 0;
}

void showSelectedParam() {
  while (switchMode == PAR_SELECTED_MODE) {
    lcd.setCursor(0, 1);
    lcd.print(params[currPos - 1].parm_name);
    if (currPos == inex_rati.index) {
      ierSelect = true;
      return;
    }
    if (currPos == peep_pres.index) {
      peepSelect = true;
      return;
    }
    params[currPos - 1].value_new_pot = analogRead(analog_pins[currPos - 1]);
    lcd.setCursor(10, 1);
    sprintf(paddedValue, "%4d", params[currPos - 1].value_new_pot);
    lcd.print(paddedValue);
    float actualValue = getCalibValue(params[currPos - 1].value_new_pot, currPos - 1);
    lcd.setCursor(10, 2);
    lcd.print(actualValue, 2);
    lcd.setCursor(10, 3);
    float storedValue = getCalibValue(params[currPos - 1].value_curr_mem, currPos - 1);
    lcd.print(storedValue);
    delay(mode_loop_delays[switchMode]);
  }
}

void saveSelectedParam() {
  if (switchMode == PAR_SAVE_MODE) {
    storeParam(params[currPos - 1]);
    params[currPos - 1].value_curr_mem = params[currPos - 1].value_new_pot;
    switchMode = DISPLAY_MODE;
    actionPending = 0;
    ierSelect = false;
    lcd.setCursor(0, 4);
    lcd.print(params[currPos - 1].parm_name);
    lcd.print(" saved.....");
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
float getCalibValue(int potValue, int paramIndex) {
  float convVal = map(potValue, 0, POT_HIGH, params[paramIndex].min_val, params[paramIndex].max_val);
  return ((int)(convVal / params[paramIndex].incr) + 1) * params[paramIndex].incr;
}

/*
   The below method is for a specific parameter
*/
float getCalibratedParam(ctrl_parameter_t param) {
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
