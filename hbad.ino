#include <Wire.h>
#include "pinout.h"
#include "ctrl_display.h"
//#include "lcd.h"
#include "hbad_serial.h"
#include "calib_calc_m_c.h"
#include "sensor_params.h"
#include "sensor_read.h"
#include "./libraries/MsTimer2/MsTimer2.h"
#include "./libraries/MsTimer2/MsTimer2.cpp"
#include "Service_Mode.h"
#include "Control_StateMachine.h"
int TimeSeries =0;
volatile short currPos = 1;
unsigned short newIER = 1;
unsigned short newPeep = 5;

long int resetEditModetime;
int ctrlParamChangeInit = 0;
volatile int switchMode = 0;
volatile boolean actionPending = false;
int lastCLK;
int currentCLK;
String saveFlag = "Save  ";
String cancelFlag = "Cancel";
int currentSaveFlag = 1;
int gCtrlParamUpdated = 0;
#define ROT_ENC_FOR_IER (currPos == inex_rati.index)
#define ROT_ENC_FOR_PEEP (currPos == peep_pres.index)

#define READ_FROM_ENCODER ((switchMode == PAR_SELECTED_MODE) \
                           && (currPos >=0 && (currPos < MAX_CTRL_PARAMS) \
                               && (params[currPos].readPortNum == DISP_ENC_CLK)))

#define LCD_DISP_REFRESH_COUNT 5
#define EDIT_MODE_TIMEOUT 5000
#define SEND_CTRL_PARAMS_COUNT 15
#define SEND_SENSOR_VALUES_COUNT 5


int lcdRunTimerRefreshCount = 0;
int ContrlParamsSendCount = 0;
int SensorValuesSendCount = 0;

// Control statemachine gloabl variable
ControlStatesDef_T geCtrlState = CTRL_INIT;
ControlStatesDef_T geCtrlPrevState  = CTRL_INIT;
bool bSendInitCommand = false;
bool machineOn = false;
//Need to Integrate into Main Code
bool compressionCycle = false;
bool expansionCycle = false;
bool homeCycle = false;
String rxdata;
int comcnt;
#define SYNCH "SY"
#define ALLPARAM "AA"
#define VENTSLAVE "VS"
#define ALLSENSORS "SS"
#define SINGLEPARAM "SP"

bool gCntrlSerialEventRecvd = false;
bool bSendPeakHighDetected = false;
bool bSendPeepLowDetected = false;
bool bBreathDetectedFlag = false;

byte editSeletIndicator = 0;
byte editScrollIndex = 0;
bool menuChanged = false;
bool editSelectionMade = false;
#define CYLINDER 0
#define HOSP_LINE 1
bool o2LineSelect = CYLINDER; 
bool tempO2LineSelect = 0;
char * o2LineString[2] = {"Cylinder", "Hospital line"};
bool o2LineChange = false;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_SWITCH, INPUT_PULLUP);
  pinMode(DISP_ENC_CLK, INPUT);
  pinMode(DISP_ENC_DT, INPUT);
  pinMode(DISP_ENC_SW, INPUT_PULLUP);
  pinMode(ADS115_INT_PIN, INPUT_PULLUP);
  lcd.begin(LCD_LENGTH_CHAR, LCD_HEIGHT_CHAR);
  Wire.setClock(4000000L);
  Wire.begin();
  hbad_mem.begin();
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  //attachInterrupt(digitalPinToInterrupt(DISP_ENC_SW), isr_processStartEdit, HIGH);
  getAllParamsFromMem();
  setup_calib_calc_m_c();
  setup_service_mode();
  displayInitialScreen();
  displayO2settingScreen();
  MsTimer2::set(120, saveSensorData);
  MsTimer2::start();
  Serial.println("Exiting setup !");
}
boolean runInitDisplay = true;
void displayRunTime()
{
  if (runInitDisplay)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("       STATUS");
    runInitDisplay = false;
  }
  // cleanRow(1); cleanRow(2); cleanRow(3);
  String row1 = "TV:";
  row1 += params[TIDAL_VOL].value_curr_mem;
  while (row1.length() < 6)
  {
    row1 += " ";
  }
  row1 += "mL      BPM:";
  row1 += params[BPM].value_curr_mem;

  while (row1.length() < 20)
  {
    row1 += " ";
  }
  lcd.setCursor(0, 1);
  lcd.print(row1);

  String row2 = "FiO2:";
  row2 += (PS_ReadSensorValueX10(O2)) / 10;
  row2 += "%";
  //Serial.println((PS_ReadSensorValueX10(O2)) / 10);
  while (row2.length() < 8)
  {
    row2 += " ";
  }
  row2 += "     IER:1:";
  row2 += params[IE_RATIO].value_curr_mem;
  lcd.setCursor(0, 2);
  lcd.print(row2);


  String row3 = "PEEP:";
  row3 += params[PEEP_PRES].value_curr_mem;

  while (row3.length() < 7)
  {
    row3 += " ";
  }

  row3 += "  IP:";
  row3 += PS_ReadSensorValueX10(PS1) / 10;
  while (row3.length() < 14)
  {
    row3 += " ";
  }

  row3 += " EP:";
  row3 += PS_ReadSensorValueX10(PS2) / 10;
  while (row3.length() < 20)
  {
    row3 += " ";
  }
  lcd.setCursor(0, 3);
  lcd.print(row3);
}

void serialPrintToPlotter()
{
  Serial.print((PS_ReadSensorValueX10(O2)) / 10);
  Serial.print(" ");
  Serial.print((PS_ReadSensorValueX10(PS1)) / 10);
  Serial.print(" ");
  Serial.print((PS_ReadSensorValueX10(PS2)) / 10);
  Serial.println(" ");
}

#define O2_MIN_DIFF 5
#define O2_MAX_DIFF 10
#define PS1_MIN_LIMIT 10
#define PS1_LEAK_BPM_MONITOR 2

#define ALARM_O2_RANGE      0x1
#define ALARM_PRESSURE_LEAK 0x2

void checkAlarms()
{
  unsigned int raiseAlarm = 0;
  static unsigned long PsAlarmDetectStartTime=0;
  //O2 check
  if ((((PS_ReadSensorValueX10(O2)) / 10) < (params[FIO2_PERC].value_curr_mem - O2_MIN_DIFF)) ||
      (((PS_ReadSensorValueX10(O2)) / 10) > (params[FIO2_PERC].value_curr_mem + O2_MAX_DIFF)))
  {
    raiseAlarm |= ALARM_O2_RANGE;
  }
  //pressure sensors inhale and exhale
  if ((PS_ReadSensorValueX10(PS1) < PS_ReadSensorValueX10(PS2))||
       (PS_ReadSensorValueX10(PS1) < PS1_MIN_LIMIT) ||
       (PS_ReadSensorValueX10(PS1) < params[PEEP_PRES].value_curr_mem))
  {
    /* 
     * this can happen in exhale/hold cycle. but not on inhale cycle 
     * check if this is consistent for entire breath
     */
    
    if (PsAlarmDetectStartTime == 0)
    {
      PsAlarmDetectStartTime = millis();//note current time
    }
    else if ((millis - PsAlarmDetectStartTime) > (PS1_LEAK_BPM_MONITOR *60 * 1000 / params[BPM].value_curr_mem))
    {
      raiseAlarm |= ALARM_PRESSURE_LEAK;
    }
    
  }
  else
  {
    PsAlarmDetectStartTime = 0;
  }
}

/* Project Main loop */

void loop() {
  RT_Events_T eRTState;
  if (gSensorDataUpdated == 1)
  {
    //checkForPs2Dip();
    displayRunTime();
    serialPrintToPlotter();
    checkSendDataToGraphicsDisplay();
    checkAlarms();
    gSensorDataUpdated= 0;
  }

  eRTState = encoderScanUnblocked();
  if (eRTState == RT_BT_PRESS)
  {
    if (millis() - resetEditModetime > 500)
    {
      Serial.println("Entering Edit mode!");
      MsTimer2::stop();
      resetEditModetime = millis();
      displayEditMenu();
      gCtrlParamUpdated = 1;
      editSeletIndicator = 0;
      editScrollIndex = 0;
      MsTimer2::start();
      runInitDisplay = true;
      resetEditModetime = millis();
    }
  }
  if (gCntrlSerialEventRecvd == true)
  {
    gCntrlSerialEventRecvd = false;
    Ctrl_ProcessRxData();
  }
  Ctrl_StateMachine_Manager();
  if (digitalRead(RESET_SWITCH) == LOW)
  {
    //reset switch.
   // bSendInitCommand = true;
    if(machineOn){
      Serial2.print(commands[STPR_STP]);
      machineOn = false;
      bSendInitCommand = false;
    }else{
      machineOn = true;
       bSendInitCommand = true;
       geCtrlState = CTRL_INIT;
    }
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
  }
 // getGraphSensorsReading();
}


typedef enum {
  E_EXIT,
  E_TV,
  E_BPM,
  E_FiO2,
  E_IER,
  E_PEEP,
  E_PIP,
  E_O2_INPUT,
  MAX_EDIT_MENU_ITEMS
} editMenu_T;

char* mainEditMenu[MAX_EDIT_MENU_ITEMS] = {"EXIT EDIT MENU", "TV", "BPM", "FiO2", "IER", "PEEP", "PIP", "O2 in"};
editMenu_T currentEditMenuIdx = MAX_EDIT_MENU_ITEMS;



void printEditMenu( void)
{
  String strOnLine234;
#if SERIAL_PRINTS
  Serial.println(editSeletIndicator);
  Serial.println(editScrollIndex);
#endif
  lcd.clear();
  for (int i = 0; i <= LCD_HEIGHT_CHAR - 1; i++) //menuItems[menuIdx].menuLength; i++)
  {
    lcd.setCursor(0, i);
    if (editSeletIndicator == i)
    {
      strOnLine234 = ">";
    }
    else
    {
      strOnLine234 = " ";
    }
    strOnLine234 += mainEditMenu[editScrollIndex + i];
    if (editScrollIndex + i != 0)
    {
      strOnLine234 += ":";
    }
    switch (editScrollIndex + i)
    {
      case (E_TV):
        strOnLine234 += params[TIDAL_VOL].value_curr_mem;
        strOnLine234 += "mL";
        break;
      case (E_BPM):
        strOnLine234 += params[BPM].value_curr_mem;
        break;
      case (E_FiO2):
        strOnLine234 += params[FIO2_PERC].value_curr_mem;
        strOnLine234 += "%";
        break;
      case (E_IER):
        strOnLine234 += " 1:";
        strOnLine234 += params[IE_RATIO].value_curr_mem;
        break;
      case (E_PEEP):
        strOnLine234 += params[PEEP_PRES].value_curr_mem;
        strOnLine234 += "cmH2O";
        break;
      case (E_PIP):
        strOnLine234 += params[PEAK_PRES].value_curr_mem;
        strOnLine234 += "cmH2O";
        break;
      case (E_O2_INPUT):
        strOnLine234 += o2LineString[o2LineSelect];
        break;
    }

    lcd.print (strOnLine234);
#if SERIAL_PRINTS
    Serial.println(strOnLine234);
#endif
  }
}


void moveUpEdit()
{
  editSeletIndicator++;
  /*
     check wrap around of select indicator
  */
  if ((editSeletIndicator >= MAX_EDIT_MENU_ITEMS) ||
      (editSeletIndicator > LCD_HEIGHT_CHAR - 1))
  {
    editSeletIndicator = min(LCD_HEIGHT_CHAR - 1, MAX_EDIT_MENU_ITEMS);
    if (editSeletIndicator == LCD_HEIGHT_CHAR - 1)
    {
      editScrollIndex++;
    }
    if ((editScrollIndex + editSeletIndicator) >= MAX_EDIT_MENU_ITEMS)
    {
      editScrollIndex = MAX_EDIT_MENU_ITEMS - editSeletIndicator - 1;
    }
  }
}

void moveDownEdit()
{
  /*
     check wrap around of select indicator
  */
  if (editSeletIndicator == 0 )
  {
    if (editScrollIndex > 0)
    {
      editScrollIndex--;
    }
  }
  else
  {
    editSeletIndicator--;
  }
}

void selectionEdit()
{
  //  if ((editSeletIndicator + editScrollIndex) != MAX_EDIT_MENU_ITEMS)
  //  {
  //    //we are already in intial edit menu
  //    lcd.clear();
  //    lcd.setCursor(0, 1);
  //    lcd.print("You selected:");
  //    lcd.setCursor(0, 2);
  //    lcd.print(mainEditMenu[editSeletIndicator + editScrollIndex]);
  //    delay(1000);
  //    lcd.clear();
  //  }
  lcd.clear();
  editSelectionMade = true;
}



void editModeEncoderInput(void)
{
  RT_Events_T eRTState = RT_NONE;
  eRTState = encoderScanUnblocked();
  switch (eRTState)
  {
    case RT_INC:
      moveUpEdit();
      break;
    case   RT_DEC:
      moveDownEdit();
      break;
    case   RT_BT_PRESS:
      selectionEdit();
      break;
  }
  if (eRTState != RT_NONE)
  {
    resetEditModetime = millis();
    menuChanged = true;
  }
}

long unsigned lastDisplayTime = 0;
int oldValue;
void showSaveSelectedParam()
{
  RT_Events_T eRTState = RT_NONE;
  eRTState = encoderScanUnblocked();
  switch (eRTState)
  {
    case RT_INC:
      currentSaveFlag = false;
      break;
    case   RT_DEC:
      currentSaveFlag = true;
      break;
    case   RT_BT_PRESS:
      switchMode = PAR_SAVE_MODE;
      saveSelectedParam();
      editSelectionMade = false;
      break;
  }
  if (eRTState != RT_NONE)
  {
    resetEditModetime = millis();
  }
  if ((millis() - lastDisplayTime > 500) ||
      (eRTState != RT_NONE))
  {
    lastDisplayTime = millis();
    //Serial.println("in showSaveSelectedParam");
    //Serial.println(currPos);
    if (o2LineChange)
    {
      // change tempO2LineSelect
      o2LineChanger(EDIT_MODE_TIMEOUT);
    }
    else
    {
      lcd.setCursor(8, 0);
      lcd.print(params[currPos].parm_name);
  
//      if (currPos == FIO2_PERC)
//      {
//        lcd.setCursor(0,2);
//        lcd.print("Set using FiO2 valve");
//      }
//      else 
      if (currPos == PEEP_PRES)
      {
        lcd.setCursor(0,2);
        lcd.print("Set using PEEP valve");
      }
      else
      {
        if (currPos == inex_rati.index || currPos == peep_pres.index) {
          abc();
          return;
        }
        params[currPos].value_new_pot = analogRead(params[currPos].readPortNum);
        lcd.setCursor(0, 1);
        lcd.print("New: ");
        lcd.print("   ");
        lcd.setCursor(8, 1);
        int actualValue = getCalibValue(params[currPos].value_new_pot, currPos);
        printPadded(actualValue);
        lcd.setCursor(15, 1);
        lcd.print(params[currPos].units);
        lcd.setCursor(0, 2);
        lcd.print("Old: ");
        lcd.setCursor(8, 2);
        printPadded(params[currPos].value_curr_mem);
        lcd.setCursor(15, 2);
        lcd.print(params[currPos].units);
        lcd.setCursor(0, 3);
        if (currentSaveFlag == true) {
          lcd.print(SAVE_FLAG_CHOSEN);
          lcd.print("    ");
          lcd.print(CANC_FLAG);
        } else {
          lcd.print(SAVE_FLAG);
          lcd.print("    ");
          lcd.print(CANC_FLAG_CHOSEN);
        }
  
        int diffValue = abs(oldValue - params[currPos].value_new_pot);
        //Serial.print("diffValue "); Serial.println(diffValue);
        if (diffValue>5)
        {
          resetEditModetime = millis();
        }
        oldValue = params[currPos].value_new_pot;
      }
    }
    //Serial.println("exiting showSaveSelectedParam");
  }
}

void displayEditMenu(void)
{
  menuChanged = true;
  bool continueEditModeFlag = true;
  editSelectionMade = false;
  currentSaveFlag = false;
  do {
    if (editSelectionMade == false)
    {
      //display prameters if changed.
      if (menuChanged)
      {
        printEditMenu();
        menuChanged = false;
      }
      //get parameters changes in unblocked manner.
      editModeEncoderInput();
    }
    else
    {
      //act on the input
      switch (editSeletIndicator + editScrollIndex)
      {
        case (E_EXIT):
          continueEditModeFlag = false;
          break;
        case (E_TV):
          currPos = TIDAL_VOL;
          break;
        case (E_BPM):
          currPos = BPM;
          break;
        case (E_FiO2):
          currPos = FIO2_PERC;
          break;
        case (E_IER):
          currPos = IE_RATIO;
          break;
        case (E_PEEP):
          currPos = PEEP_PRES;
          break;
        case (E_PIP):
          currPos = PEAK_PRES;
          break;
        case (E_O2_INPUT):
          o2LineChange = true;
          break;
        default:
          break;
      }
      if (continueEditModeFlag == true)
      {
        switchMode = PAR_SELECTED_MODE;
        showSaveSelectedParam();
      }
    }
  } while (((millis() - resetEditModetime) < EDIT_MODE_TIMEOUT) && (continueEditModeFlag));
}

void saveSelectedParam() {
  if (switchMode == PAR_SAVE_MODE) {
    //    Serial.println(params[currPos].value_new_pot);
    lcd.setCursor(0, 3);
    lcd.print(params[currPos].parm_name);
    String command;
    String param;
    if (o2LineChange)
    {
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(" saved          ");
      if (o2LineSelect != tempO2LineSelect)
      {
        o2LineSelect = tempO2LineSelect;
        if (o2LineSelect == CYLINDER)
        {
          Serial2.print(commands[OXY_SOLE_HOS_O2_OFF]);
          delay(10);
          Serial2.print(commands[OXY_SOLE_CYL_ONN]);
        }
        else //HOSP_LINE
        {
          Serial2.print(commands[OXY_SOLE_CYL_OFF]);
          delay(10);
          Serial2.print(commands[OXY_SOLE_HOS_O2_ONN]);
        }
      }
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      return;
    }

    if ((ROT_ENC_FOR_PEEP))// ||
        //(currPos == FIO2_PERC))
    {
      lcd.setCursor(0, 3);
      lcd.print("   going back....");
      delay(500);
      return;
    }

    if (ROT_ENC_FOR_IER || ROT_ENC_FOR_PEEP) {
      params[currPos].value_curr_mem = getCalibratedParamFromPot(params[currPos]);
      storeParam(params[currPos]);
      if (params[currPos].parm_name == IER) {
        param = PARAM5;
      }
      command = getSensorReading(param, params[currPos].value_curr_mem);
      Serial2.print(command);
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(" saved          ");
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      return;
    }
    if (currentSaveFlag == 0) {
      lcd.print(" Edit cancelled.");
    } else {

      params[currPos].value_curr_mem = getCalibratedParamFromPot(params[currPos]);
      storeParam(params[currPos]);
      if (params[currPos].parm_name == TIDAL_VOLUME) {
        param = PARAM1;
      } else if (params[currPos].parm_name == RR) {
        param = PARAM2;
      }
      command = getSensorReading(param, params[currPos].value_curr_mem);
      Serial2.print(command);
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(" saved.....");
      digitalWrite(BUZZER_PIN, HIGH);
    }
    actionPending = false;
    switchMode = DISPLAY_MODE;
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    cleanRow(3);
    resetEditModetime = millis();
  }
}

void isr_processStartEdit() {
  static unsigned long lastSwitchTime = 0;
  unsigned long switchTime = millis();
  if ((switchTime - lastSwitchTime) < DBNC_INTVL_SW) {
    return;
  }
  switchMode = (switchMode + 1) % 4;
  resetEditModetime = millis();
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
int getCalibratedParamFromPot(ctrl_parameter_t param) {
  if (param.readPortNum == DISP_ENC_CLK) {
    return param.value_curr_mem;
  }
  float convVal = map(param.value_new_pot, 0, POT_HIGH, param.min_val, param.max_val);
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


void displayChannelData(sensor_e sensor)
{
  int o2mVReading, o2Unitx10;
  RT_Events_T eRTState = RT_NONE;
#if SERIAL_PRINTS
  Serial.println("we are in diagO2Sensor");
#endif

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Input for:");
  lcd.print(menuItems[currentMenuIdx].menu[seletIndicator + scrollIndex - 1 ]);
  lcd.setCursor(0, 1);

  lcd.print("current reading:");

  while (RT_NONE == eRTState)
  {
    lcd.setCursor(0, 2);
    o2mVReading = PS_ReadSensor( sensor );
    Serial.print(o2mVReading);
    Serial.print(DPS2);
    String disp = "";
    disp += o2mVReading;
    disp += "mV, ";
    o2Unitx10 = getSensorUnitsx10(O2, o2mVReading);
    disp += (((float)o2Unitx10) / 10);
    if (sensor == O2)
    {
      disp += ("%   ");
    }
    else
    {
      disp += ("cmH2O  ");
    }
    while (disp.length() < LCD_LENGTH_CHAR)
    {
      disp += " ";
    }
    lcd.print(disp);
    for (int wait = 0; wait < 200; wait += 20)
    {
      eRTState = encoderScanUnblocked();
      if (eRTState != RT_NONE)
      {
        break;
      }
      delay (20);
    }
  }
  switch (eRTState)
  {
    case RT_INC:
    case RT_DEC:
#if SERIAL_PRINTS
      Serial.println("leave without changes from diagO2Sensor");
#endif
      break;
    case   RT_BT_PRESS:
      //diagSaveCalibData(O2,o2mVReading, o2Unitx10);
#if SERIAL_PRINTS
      Serial.println("save diagO2Sensor");
#endif
      break;
  }
}

void diagO2Sensor(void)
{
  displayChannelData(O2);
}
void diagAds1115(void)
{
  displayChannelData(PS1);
  displayChannelData(PS2);
  displayChannelData(DPS1);
  displayChannelData(DPS2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ads1115 validated");
  delay(2000);
}
void diagSolStatus(void)
{
  //  Serial.println("we are in diagSolStatus");
}



void displayO2settingScreen(void)
{
  float SensorVolt;
  int displayCounter = 0xFF;
  RT_Events_T eRTState = RT_NONE;

  //select O2 input line.
  o2LineChanger(0xFFFF);//large timeout.

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" Turn on the system ");
  lcd.setCursor(0,1);
  lcd.print("  press red button  ");
  lcd.setCursor(0,2);
  lcd.print("Don't yet connect to");
  lcd.setCursor(0,3);
  lcd.print("      PATIENT       ");

  while (digitalRead(RESET_SWITCH) != LOW);
  
  if(!machineOn){
    machineOn = true;
    Serial2.print(commands[INIT_MASTER]);
  }
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);
    
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set O2 level & match");
  lcd.setCursor(0,1);
  lcd.print("press knob when done");

  while (eRTState != RT_BT_PRESS)
  {
    eRTState = encoderScanUnblocked();
    switch (eRTState)
    {
      case   RT_BT_PRESS:
        currentSaveFlag = true;
        switchMode = PAR_SAVE_MODE;
        currPos = FIO2_PERC;
        saveSelectedParam();
        break;
    }
    SensorVolt=ADC_ReadVolageOnATMega2560(OXYGEN_ANALOG_PIN);
    int o2Percentage = (int) getX(SensorVolt + ((float)O2_FUDGE/1000),
              sensorData[O2].m,
              sensorData[O2].c);
    if (o2Percentage < 0 )
      o2Percentage = 0;
    if (o2Percentage > 99)
      o2Percentage = 99;
    params[FIO2_PERC].value_new_pot = analogRead(params[FIO2_PERC].readPortNum);
    int actualValue = getCalibValue(params[FIO2_PERC].value_new_pot, FIO2_PERC);
    lcd.setCursor(0,2);
    lcd.print("New:");
    lcd.print(actualValue);
    lcd.print("% ");
    lcd.setCursor(10,2);
    lcd.print("Old:");
    lcd.print(params[FIO2_PERC].value_curr_mem);
    lcd.print("% ");
    if (displayCounter > 100)
    {
      lcd.setCursor(0,3);
      lcd.print("measured:");
      lcd.print(o2Percentage);
      lcd.print("% ");
      displayCounter = 0;
    }
    displayCounter++;
  }
}


void checkSendDataToGraphicsDisplay(void)
{
  /*Send control set parameters to graphics module */
  ContrlParamsSendCount++;
  if (ContrlParamsSendCount > SEND_CTRL_PARAMS_COUNT)
  {
    ContrlParamsSendCount = 0;
    UART3_SendDAQDataGraphicDisplay(PARAMS_DATA);
  }
  /*Send Sensor Values parameters to graphics module */
  //SensorValuesSendCount++;
  //if (SensorValuesSendCount > SEND_SENSOR_VALUES_COUNT)
  //{
    SensorValuesSendCount = 0;
    UART3_SendDAQDataGraphicDisplay(SENSORS_DATA);
  //}
}

void UART3_SendDAQDataGraphicDisplay(UartPacketTypeDef ePacketType)
{
  int sendDataLen = 0;
  unsigned short int crc16Val = 0;
  int index=0;
  if (ePacketType == SENSORS_DATA)
  {
    TimeSeries++;
    u8TxBuffer[index++] = 0;
    u8TxBuffer[index++] = SENSORS_DATA_FC;
    u8TxBuffer[index++] = TOTAL_NUMBER_OF_SENSORS;
    for (int i = 0; i < TOTAL_NUMBER_OF_SENSORS; i++)
    {
      u8TxBuffer[index++] = (unsigned char)(((sensorOutputData[i].unitX10 & 0xFF00) >> 8) & 0x00FF);
      u8TxBuffer[index++] = (unsigned char)(sensorOutputData[i].unitX10 & 0x00FF);
    }
    u8TxBuffer[index++] = ((TimeSeries >>8) & 0xFF);
    u8TxBuffer[index++] = (TimeSeries & 0xFF);
  }
  else if (ePacketType == PARAMS_DATA)
  {
    u8TxBuffer[index++] = TOTAL_PARAMS_PACKET_LEN;
    u8TxBuffer[index++] = PARAMETERS_DATA_FC;
    u8TxBuffer[index++] = TOTAL_NUMBER_OF_PARAMS;
    for (int i = 0; i < TOTAL_NUMBER_OF_PARAMS; i++)
    {
      u8TxBuffer[index++] = (unsigned char)(((params[i].value_curr_mem & 0xFF00) >> 8) & 0x00FF);
      u8TxBuffer[index++] = (unsigned char)(params[i].value_curr_mem & 0x00FF);
    }
  }
  else
  {
    sendDataLen = 0;
    /*packet type unknown .. ignore it*/
  }

  if (index != 0)
  {
    u8TxBuffer[0] = index+2;
    crc16Val = crc16(u8TxBuffer, index);
    u8TxBuffer[index++] = (unsigned char)(((crc16Val & 0xFF00) >> 8) & 0x00FF);
    u8TxBuffer[index++] = (unsigned char)(crc16Val & 0x00FF);
    Serial3.write(u8TxBuffer, index);
    /*Serial.println("Graphics packet");
    for(int i=0; i<index; i++)
    {
      Serial.print(String(u8TxBuffer[i])+ " ");
    }
    Serial.println(" ");
    */
  }

}

String rxdata_buff;
void serialEvent2() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (inChar == '$') {
      comcnt = 1;
      rxdata_buff = "";
    }
    if  (comcnt >= 1) {
      rxdata_buff += inChar;
      comcnt = comcnt + 1;
      if (inChar == '&') {
        if (comcnt >= 10) {
          rxdata = rxdata_buff;
          gCntrlSerialEventRecvd = true;
        }
      }
    }
  }
}

void Ctrl_ProcessRxData(void) {
  String p1;
  String p2;
  String p3;
  String p4;
  String payload;
  String command;

  p1 = rxdata.substring(1, 3);
  p2 = rxdata.substring(3, 5);
  p3 = rxdata.substring(5, 7);
  p4 = rxdata.substring(7, 9);
  payload = p3 + p4;
  // int index = p3.toInt();
  int value;
 // Serial.println(rxdata);
  if (p1 == VENTSLAVE) {
    if (p2 == SINGLEPARAM ) {

    }
    else if (p2 == SYNCH) {
      //      Serial.println(rxdata);
      geCtrlState = payload.toInt();
    }
    else if (p2 == ALLSENSORS) {

    }
    else if (p2 == ALLPARAM) {

    }
    else {
      int index;
      index =  payload.toInt();

      if (index < MAX_CTRL_PARAMS)
      {
        value = params[index].value_curr_mem;
        command = getSensorReading(p2, value);
        Serial2.print(command);
   //     Serial.print(command);
      }

    }

  }

}
/*
   Function to send  specific the Calibrated Sensor Reading
*/
String getSensorReading(String paramName, int value) {
  String command;
  char paddedValue[3];
  command = START_DELIM;
  command += VENT_MAST;
  command += paramName;
  sprintf(paddedValue, "%04d",
          value);
  command += paddedValue;
  command += END_DELIM;
  return command;
}

void Ctrl_StateMachine_Manager(void)
{
  bool stateChanged = false;
  switch (geCtrlState)
  {
    case CTRL_INIT:
      {
        if (bSendInitCommand == true)
        {
          bSendInitCommand = false;
          Serial2.print(commands[INIT_MASTER]);
          for (int index = 0; index < MAX_PS2_SAMPLES; index++)
          {
            ps2Samples[index] = 0xFFFF;
          }
        }
      }
      break;
    case CTRL_COMPRESSION:
      {
        /*When Peak Pressure Set in the UI is less than the sensor measured Peak PressureValue*/
        if (sensorOutputData[PS1].unitX10 > ((params[PEAK_PRES].value_curr_mem -5) * 10) )
        {
          if (bSendPeakHighDetected == false)
          {
            bSendPeakHighDetected = true;
            Serial2.print(commands[INH_SOLE_OFF]);
          }
        }
      }
      break;
    case CTRL_COMPRESSION_HOLD:
      {

      }
      break;
    case CTRL_EXPANSION:
      {
        /*When Peak Pressure Set in the UI is less than the sensor measured Peak PressureValue*/
        if (sensorOutputData[PS2].unitX10 < (params[PEEP_PRES].value_curr_mem * 10) )
        {
          if (bSendPeepLowDetected == false)
          {
            bSendPeepLowDetected = true;
            Serial2.print(commands[EXH_SOLE_OFF]);
          }
        }
      }
      break;
    case CTRL_EXPANSION_HOLD:
      {

      }
      break;
    case CTRL_INHALE_DETECTION:
      {
        if (bBreathDetectedFlag == false)
        {
          if (checkForPs2Dip())
          {
            bBreathDetectedFlag = true;
            Serial2.print(commands[INIT_BREATH_DET]);
          }
        }
      }
      break;
    case CTRL_UNKNOWN_STATE:
      {

      }
      break;
    default:
      break;
  }
  if (geCtrlPrevState != geCtrlState)
  {
    geCtrlPrevState = geCtrlState;
    bSendInitCommand = false;
    bSendPeakHighDetected = false;
    bSendPeepLowDetected = false;
    bBreathDetectedFlag = false;
  }
}

void o2LineChanger(int timeoutValue) {
  RT_Events_T eRTState;
  lastDisplayTime = millis();
  resetEditModetime = millis();
  lcd.clear();
  do {
    eRTState = encoderScanUnblocked();
    if (eRTState != RT_NONE)
    {
      resetEditModetime = millis();
    }
    switch (eRTState)
    {
      case RT_INC:
        tempO2LineSelect = 1;
        break;
      case   RT_DEC:
        tempO2LineSelect = 0;
        break;
      case   RT_BT_PRESS:
        currentSaveFlag = true;
        switchMode = PAR_SAVE_MODE;
        saveSelectedParam();
        editSelectionMade = false;
        o2LineChange = false;
        return;
        break;
    }
    if ((millis() - lastDisplayTime > 500) ||
      (eRTState != RT_NONE))
    {
      lcd.setCursor(0,0);
      lcd.print("select O2 input line");
      lcd.setCursor(0, 1);
      if (tempO2LineSelect == 0)
      {
        lcd.print("< O2 Cylinder >  ");
        lcd.setCursor(0, 2);
        lcd.print("  Hospital Line  ");
      }
      else
      {
        lcd.print("  O2 Cylinder    ");
        lcd.setCursor(0, 2);
        lcd.print("< Hospital Line > ");
      }
      lastDisplayTime = millis();
    }
  }while ((millis() - resetEditModetime) < timeoutValue);
}

void abc() {
  RT_Events_T eRTState;
  lastDisplayTime = millis();
  int oldIER = params[inex_rati.index].value_curr_mem;
  resetEditModetime = millis();
  do {
    int incr = 0;
    eRTState = encoderScanUnblocked();
    if (eRTState != RT_NONE)
    {
      resetEditModetime = millis();
    }
    switch (eRTState)
    {
      case RT_INC:
        incr++;
        break;
      case   RT_DEC:
        incr--;
        break;
      case   RT_BT_PRESS:
        currentSaveFlag = true;
        switchMode = PAR_SAVE_MODE;
        saveSelectedParam();
        editSelectionMade = false;
        return;
        break;
    }
    if ((millis() - lastDisplayTime > 500) ||
        (incr != 0))
    {
      lastDisplayTime = millis();
      if (ROT_ENC_FOR_IER) {
        newIER = rectifyBoundaries(newIER + incr, inex_rati.min_val, inex_rati.max_val);
        //      cleanRow(1);
        ; lcd.setCursor(VALUE1_DISPLAY_POS, 1);
        lcd.setCursor(3, 1);
        lcd.print("New IER  1:");
        lcd.print(newIER);
        lcd.setCursor(3, 2);
        lcd.print("Old IER  1:");
        lcd.print(oldIER);
        params[inex_rati.index].value_new_pot = newIER;
        params[inex_rati.index].value_curr_mem = newIER;
      } else if (ROT_ENC_FOR_PEEP) {
        newPeep = rectifyBoundaries(newPeep + incr * peep_pres.incr, peep_pres.min_val, peep_pres.max_val);
        params[peep_pres.index].value_new_pot = newPeep;
        params[peep_pres.index].value_curr_mem = newPeep;
        lcd.setCursor(VALUE1_DISPLAY_POS, 1);
        printPadded(newPeep);
        lcd.setCursor(0, 3);
        lcd.print("set using PEEP valve");
      }
      incr = 0;
    }
    lastCLK = currentCLK;
  } while ((millis() - resetEditModetime) < EDIT_MODE_TIMEOUT);
}
/*
   Function to send all the Sensors Data for plotting the Graph
*/
String getGraphSensorsReading() {
  String command;
  for (int i = 0; i < MAX_CTRL_PARAMS; i++) {
    command +=  params[i].value_curr_mem;
    command += ",";
  }
  for (int i = 0; i < NUM_OF_SENSORS; i++) {
    command +=  sensorOutputData[i].unitX10;
    if (i != 4)
      command += ",";
  }
  command += "\r\n";
  //Serial3.print(command);
  return command;
}
