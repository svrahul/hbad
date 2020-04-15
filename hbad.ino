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

volatile short currPos = 1;
unsigned short newIER = 1;
unsigned short newPeep = 5;

long int resetEditModetime;
int ctrlParamChangeInit = 0;
volatile int switchMode = 0;
volatile static short announced = 0;
volatile boolean actionPending = false;
int lastCLK = 0;
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

int lcdRunTimerRefreshCount =0;
int ContrlParamsSendCount = 0;
int SensorValuesSendCount = 0;

// Control statemachine gloabl variable
ControlStatesDef_T geCtrlState = CTRL_INIT;
ControlStatesDef_T geCtrlPrevState  = CTRL_INIT;
bool bSendInitCommand = true;
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
  Serial2.begin(9600);
  Serial3.begin(9600);
  attachInterrupt(digitalPinToInterrupt(DISP_ENC_SW), isr_processStartEdit, HIGH);
  getAllParamsFromMem();
  setup_calib_calc_m_c();
  setup_service_mode();
  displayInitialScreen();
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
    lcd.print("RunTime parameters");
    runInitDisplay = false;
  }
  // cleanRow(1); cleanRow(2); cleanRow(3);
  String row1 = "TV:";
  row1 += tidl_volu.value_curr_mem;
  while (row1.length() != 6)
  {
    row1 += " ";
  }
  row1 += "RR:";
  row1 += resp_rate.value_curr_mem;
  while (row1.length() != 11)
  {
    row1 += " ";
  }
  row1 += " IER:";
  row1 += inex_rati.value_curr_mem;
  while (row1.length() != 20)
  {
    row1 += " ";
  }
  lcd.setCursor(0, 1);
  lcd.print(row1);

  String row2 = "PP:";
  row2 += peep_pres.value_curr_mem;
  while (row2.length() != 6)
  {
    row2 += " ";
  }
  row2 += "O2:";
  row2 += (PS_ReadSensorValueX10(O2)) / 10;
  while (row2.length() != 12)
  {
    row2 += " ";
  }
  row2 += "IP:";
  row2 += PS_ReadSensorValueX10(PS1) / 10;
  while (row2.length() != 20)
  {
    row2 += " ";
  }
  lcd.setCursor(0, 2);
  lcd.print(row2);


  String row3 = "EP:";
  row3 += PS_ReadSensorValueX10(PS2) / 10;
  while (row3.length() != 7)
  {
    row3 += " ";
  }
  lcd.setCursor(0, 3);
  lcd.print(row3);
} 


/* Project Main loop */
 
void loop() {
  RT_Events_T eRTState;
  if(gSensorDataUpdated ==1)
  {
    //checkForPs2Dip();
    lcdRunTimerRefreshCount++;
    if(lcdRunTimerRefreshCount == LCD_DISP_REFRESH_COUNT)
    {
      displayRunTime();
      lcdRunTimerRefreshCount = 0;
    }
    checkSendDataToGraphicsDisplay();    
  }
  
  eRTState = encoderScanUnblocked();
  if (eRTState == RT_BT_PRESS)
  {
    Serial.println("Entering Edit mode!");
    MsTimer2::stop();
    editMode();
    gCtrlParamUpdated = 1;
    MsTimer2::start();
    runInitDisplay = true;
  }
  if (gCntrlSerialEventRecvd == true)
  {
    gCntrlSerialEventRecvd = false;
    Ctrl_ProcessRxData();
  }
  Ctrl_StateMachine_Manager();
}

void editMode()
{
  lcd.clear();
  switchMode = DISPLAY_MODE;
  resetEditModetime=millis();
  do{
    //  sendCommands();
    announce();
    saveSensorData();
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
  }while ((millis() - resetEditModetime) < EDIT_MODE_TIMEOUT);
}


void announce() {
  if (announced == 0) {
    listDisplayMode();
    editParameterMode();
    //announced = 1;
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
        lcd.print(params[i].value_curr_mem);
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
        lcd.print(params[i + 1].value_curr_mem);
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
          params[inex_rati.index].value_curr_mem = newIER;
        } else if (ROT_ENC_FOR_PEEP) {
          newPeep = rectifyBoundaries(newPeep + cursorIndex * peep_pres.incr, peep_pres.min_val, peep_pres.max_val);
          params[peep_pres.index].value_new_pot = newPeep;
          params[peep_pres.index].value_curr_mem = newPeep;
          lcd.setCursor(VALUE1_DISPLAY_POS, 1);
          if(newPeep<10){
            lcd.print(" ");
          }
          lcd.print(newPeep);
          retVal = newPeep;
        }
      } else {
        currPos = rectifyBoundaries(currPos + cursorIndex, 0, MAX_CTRL_PARAMS - 1);
      }
      resetEditModetime=millis();
    }
    retVal = currPos;
    lastCLK = currentStateCLK;
    lastRotateTime = rotateTime;
  }
  return retVal;
}

void showSelectedParam() {
  int currentStateCLK = digitalRead(DISP_ENC_CLK);

  while (switchMode == PAR_SELECTED_MODE) {
    actionPending = true;
    lcd.setCursor(NAME1_DISPLAY_POS, 1);
    lcd.print(params[currPos].parm_name);
    if (currPos == inex_rati.index || currPos == peep_pres.index) {
      return;
    }
    params[currPos].value_new_pot = analogRead(params[currPos].readPortNum);
    lcd.setCursor(VALUE1_DISPLAY_POS, 1);
    int actualValue = getCalibValue(params[currPos].value_new_pot, currPos);
    lcd.setCursor(VALUE1_DISPLAY_POS, 1);
    if(actualValue<100){
      lcd.print(" ");
    }
    lcd.print(actualValue);
    lcd.setCursor(15, 1);
    lcd.print(params[currPos].units);
    lcd.setCursor(VALUE1_DISPLAY_POS, 2);
//    int storedValue = getCalibValue(params[currPos].value_curr_mem, currPos);
    if(params[currPos].value_curr_mem<10){
      lcd.print("  ");
    }else if(params[currPos].value_curr_mem<100){
      lcd.print(" ");
    }
    lcd.print(params[currPos].value_curr_mem);
    lcd.setCursor(14, 3);
    currentStateCLK = digitalRead(DISP_ENC_CLK);
    if (currentStateCLK != lastCLK) {
      currentSaveFlag = 1 - currentSaveFlag;
      resetEditModetime=millis();
      return;
    }
    lastCLK = currentStateCLK;
    if (currentSaveFlag == 1) {
      lcd.print(saveFlag);
    } else {
      lcd.print(cancelFlag);
    }
    resetEditModetime=millis();
    processRotationInSelectedMode();
    delay(mode_loop_delays[switchMode]);
  }
}

void saveSelectedParam() {
  if (switchMode == PAR_SAVE_MODE) {
    //    Serial.println(params[currPos].value_new_pot);
    lcd.setCursor(0, 3);
    lcd.print(params[currPos].parm_name);
    if (currentSaveFlag == 0) {
      lcd.print(" edit cancelled.....   ");
    } else {
      params[currPos].value_curr_mem = getCalibratedParamFromPot(params[currPos]);
      storeParam(params[currPos]);
      lcd.print(" saved.....");
    }
    actionPending = false;
    switchMode = DISPLAY_MODE;
    delay(1000);
    cleanRow(3);
    resetEditModetime=millis();
  }
}

void isr_processStartEdit() {
  static unsigned long lastSwitchTime = 0;
  unsigned long switchTime = millis();
  if ((switchTime - lastSwitchTime) < DBNC_INTVL_SW) {
    return;
  }
  switchMode = (switchMode + 1) % 4;
  resetEditModetime=millis();
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
int getCalibratedParamFromPot(ctrl_parameter_t param) {
  if(param.readPortNum == DISP_ENC_CLK){
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

void processRotationInSelectedMode() {
  /*Index_select loop*/
  unsigned long lastRotateTime = 0;
  unsigned long rotateTime = millis();
  int currentStateCLK = digitalRead(DISP_ENC_CLK);
  while (switchMode == PAR_SELECTED_MODE) {
    if ((rotateTime - lastRotateTime) < DBNC_INTVL_ROT) {
      return;
    }
    rotateTime = millis();
    //Serial.println("abc");
    currentStateCLK = digitalRead(DISP_ENC_CLK);
    if (currentStateCLK != lastCLK) {
      currentSaveFlag = 1 - currentSaveFlag;
      resetEditModetime=millis();
      return;
    }
    lastCLK = currentStateCLK;
    lastRotateTime = rotateTime;
    delay(25);
  }
}


void displayChannelData(sensor_e sensor)
{
  int o2mVReading, o2Unitx10;
  RT_Events_T eRTState = RT_NONE;
  #if SERIAL_PRINTS
  Serial.println("we are in diagO2Sensor");
  #endif
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Input for:");
  lcd.print(menuItems[currentMenuIdx].menu[seletIndicator + scrollIndex -1 ]);
  lcd.setCursor(0,1);
  
  lcd.print("current reading:");

  while(RT_NONE == eRTState)
  {
    lcd.setCursor(0,2);
   o2mVReading = PS_ReadSensor( sensor );
    Serial.print(o2mVReading);
    Serial.print(DPS2);
    lcd.print(o2mVReading);
    lcd.print("mV, ");
    o2Unitx10 = getSensorUnitsx10(O2, o2mVReading);
    lcd.print(((float)o2Unitx10)/10);
    if (sensor == O2)
    {
      lcd.print("%");
    }
    else
    {
      lcd.print ("pressure units");
    }
    for(int wait = 0; wait<200; wait+=20)
    {
      eRTState = encoderScanUnblocked();
      if (eRTState != RT_NONE)
      {
        break;
      }
      delay (20);
    }
  }
  switch(eRTState)
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
  lcd.setCursor(0,0);
  lcd.print("Ads1115 validated");
  delay(2000);
}
void diagSolStatus(void)
{
    Serial.println("we are in diagSolStatus");
}

void checkSendDataToGraphicsDisplay(void)
{
  /*Send control set parameters to graphics module */
  ContrlParamsSendCount++;
  if(ContrlParamsSendCount > SEND_CTRL_PARAMS_COUNT)
  {
    ContrlParamsSendCount = 0;
    UART3_SendDAQDataGraphicDisplay(PARAMS_DATA);
  }    
  /*Send Sensor Values parameters to graphics module */
  SensorValuesSendCount++;
  if(SensorValuesSendCount > SEND_SENSOR_VALUES_COUNT)
  {
    SensorValuesSendCount = 0;
    UART3_SendDAQDataGraphicDisplay(SENSORS_DATA);
  }
}

void UART3_SendDAQDataGraphicDisplay(UartPacketTypeDef ePacketType)
 {
  int sendDataLen = 0;
  unsigned short int crc16Val=0;
  if(ePacketType == SENSORS_DATA)
  {
    u8TxBuffer[PACKET_LEN_INDEX] = TOTAL_SENSORS_PACKET_LEN;
    u8TxBuffer[FUNCTION_CODE_INDEX] = SENSORS_DATA_FC;
    u8TxBuffer[NUMBER_ELEMENTS_INDEX] = TOTAL_NUMBER_OF_SENSORS;
    for(int i=0; i<TOTAL_NUMBER_OF_SENSORS; i++)
    {
      u8TxBuffer[DATA_PAYLOAD_INDEX+i*2] = (unsigned char)(((sensorOutputData[i].unitX10 & 0xFF00)>>8) & 0x00FF);
      u8TxBuffer[DATA_PAYLOAD_INDEX+i*2+1] = (unsigned char)(sensorOutputData[i].unitX10 & 0x00FF);
    }
    sendDataLen = TOTAL_SENSORS_PACKET_LEN;
  }
  else if(ePacketType == PARAMS_DATA)
  {
    u8TxBuffer[PACKET_LEN_INDEX] = TOTAL_PARAMS_PACKET_LEN;
    u8TxBuffer[FUNCTION_CODE_INDEX] = PARAMETERS_DATA_FC;
    u8TxBuffer[NUMBER_ELEMENTS_INDEX] = TOTAL_NUMBER_OF_PARAMS;
    for(int i=0; i<TOTAL_NUMBER_OF_PARAMS; i++)
    {
      u8TxBuffer[DATA_PAYLOAD_INDEX+i*2] = (unsigned char)(((params[i].value_curr_mem & 0xFF00)>>8) & 0x00FF);
      u8TxBuffer[DATA_PAYLOAD_INDEX+i*2+1] = (unsigned char)(params[i].value_curr_mem & 0x00FF);
    }
    sendDataLen = TOTAL_PARAMS_PACKET_LEN;    
  }
  else
  {
    sendDataLen = 0;
    /*packet type unknown .. ignore it*/
  }

  if(sendDataLen != 0)
  {
    crc16Val = crc16(u8TxBuffer, u8TxBuffer[PACKET_LEN_INDEX]-2);
    u8TxBuffer[SENSORS_CRC_2B_INDEX] = (unsigned char)(((crc16Val & 0xFF00)>>8) & 0x00FF);
    u8TxBuffer[SENSORS_CRC_2B_INDEX+1] = (unsigned char)(crc16Val & 0x00FF);
    Serial3.write(u8TxBuffer, sendDataLen);
  }

}

void serialEvent2() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (inChar == '$') {
      comcnt = 1;
      rxdata = "";
    }
    if  (comcnt >= 1) {
      rxdata += inChar;
      comcnt = comcnt + 1;
      if (inChar == '&') {
        if (comcnt >= 10) {
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
  if (p1 == VENTSLAVE) {
    if (p2 == SINGLEPARAM ) {

    }
    else if (p2 == SYNCH) {
      Serial.println(rxdata);
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
            ps2Samples[index]= 0xFFFF;
          }
        }
      }
      break;
    case CTRL_COMPRESSION:
      {
        /*When Peak Pressure Set in the UI is less than the sensor measured Peak PressureValue*/
        if (sensorOutputData[PS1].unitX10 > (params[PEAK_PRES].value_curr_mem * 10) )
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
