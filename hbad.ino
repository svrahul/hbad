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

byte editSeletIndicator = 0;
byte editScrollIndex = 0;
bool menuChanged = false;
bool editSelectionMade = false;

void setup() {

  pinMode(DISP_ENC_CLK, INPUT);
  pinMode(DISP_ENC_DT, INPUT);
  pinMode(DISP_ENC_SW, INPUT_PULLUP);
  pinMode(ADS115_INT_PIN, INPUT_PULLUP);
  lcd.begin(LCD_LENGTH_CHAR, LCD_HEIGHT_CHAR);
  Wire.setClock(4000000L);
  Wire.begin();
  hbad_mem.begin();
  Serial.begin(19200);
  Serial2.begin(19200);
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
    lcd.print("       STATUS");
    runInitDisplay = false;
  }
  // cleanRow(1); cleanRow(2); cleanRow(3);
  String row1 = "TV:";
  row1 += params[TIDAL_VOL].value_curr_mem;
  while (row1.length() != 6)
  {
    row1 += " ";
  }
  row1 += "mL      BPM:";
  row1 += params[BPM].value_curr_mem;

  while (row1.length() != 20)
  {
    row1 += " ";
  }
  lcd.setCursor(0, 1);
  lcd.print(row1);

  String row2 = "FiO2:";
  row2 += (PS_ReadSensorValueX10(O2)) / 10;
  row2 += "%";
  //Serial.println((PS_ReadSensorValueX10(O2)) / 10);
  while (row2.length() != 8)
  {
    row2 += " ";
  }
  row2 += "     IER:1:";
  row2 += params[IE_RATIO].value_curr_mem;
  lcd.setCursor(0, 2);
  lcd.print(row2);
  

  String row3 = "PEEP:";
  row3 += params[PEEP_PRES].value_curr_mem;

  while (row3.length() != 7)
  {
    row3 += " ";
  }

  row3 += "  IP:";
  row3 += PS_ReadSensorValueX10(PS1) / 10;
  while (row3.length() != 14)
  {
    row3 += " ";
  }

  row3 += " EP:";
  row3 += PS_ReadSensorValueX10(PS2) / 10;
  while (row3.length() != 20)
  {
    row3 += " ";
  }
  lcd.setCursor(0, 3);
  lcd.print(row3);
}


/* Project Main loop */

void loop() {
  RT_Events_T eRTState;
  if (gSensorDataUpdated == 1)
  {
    //checkForPs2Dip();
    lcdRunTimerRefreshCount++;
    if (lcdRunTimerRefreshCount == LCD_DISP_REFRESH_COUNT)
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
    displayEditMenu();
    gCtrlParamUpdated = 1;
    editSeletIndicator = 0;
    editScrollIndex = 0;
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


typedef enum {
  E_EXIT,
  E_TV,
  E_BPM,
  E_FiO2,
  E_IER,
  E_PEEP,
  E_PIP,
  MAX_EDIT_MENU_ITEMS
} editMenu_T;

char* mainEditMenu[MAX_EDIT_MENU_ITEMS] = {"EXIT EDIT MENU", "TV", "BPM", "FiO2", "IER", "PEEP", "PIP"};
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
    lcd.setCursor(8, 0);
    lcd.print(params[currPos].parm_name);

    if (currPos == FIO2_PERC)
    {
      lcd.setCursor(0,2);
      lcd.print("Set using FiO2 valve");
    }
    else if (currPos == PEEP_PRES)
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
      Serial.print("diffValue "); Serial.println(diffValue);
      if (diffValue>5)
      {
        resetEditModetime = millis();
      }
      oldValue = params[currPos].value_new_pot;
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

    if ((ROT_ENC_FOR_PEEP)||
        (currPos == FIO2_PERC))
    {
      lcd.setCursor(0, 3);
      lcd.print("   going back....");
      delay(500);
      return;
    }
    
    if (ROT_ENC_FOR_IER || ROT_ENC_FOR_PEEP) {
      params[currPos].value_curr_mem = getCalibratedParamFromPot(params[currPos]);
      storeParam(params[currPos]);
       if(params[currPos].parm_name ==IER){
        param = PARAM5;
      }
       command = getSensorReading(param,params[currPos].value_curr_mem);
      Serial2.print(command);
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(" saved          ");
      delay(500);
      return;
    }
    if (currentSaveFlag == 0) {
      lcd.print(" Edit cancelled.");
    } else {
      
      params[currPos].value_curr_mem = getCalibratedParamFromPot(params[currPos]);
      storeParam(params[currPos]);
      if(params[currPos].parm_name ==TIDAL_VOLUME){
        param = PARAM1;
      }else if(params[currPos].parm_name ==RR){
        param = PARAM2;
      }
      command = getSensorReading(param,params[currPos].value_curr_mem);
      Serial2.print(command);
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(" saved.....");
    }
    actionPending = false;
    switchMode = DISPLAY_MODE;
    delay(500);
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
    while (disp.length() != LCD_LENGTH_CHAR)
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
  SensorValuesSendCount++;
  if (SensorValuesSendCount > SEND_SENSOR_VALUES_COUNT)
  {
    SensorValuesSendCount = 0;
    UART3_SendDAQDataGraphicDisplay(SENSORS_DATA);
  }
}

void UART3_SendDAQDataGraphicDisplay(UartPacketTypeDef ePacketType)
{
  int sendDataLen = 0;
  unsigned short int crc16Val = 0;
  if (ePacketType == SENSORS_DATA)
  {
    u8TxBuffer[PACKET_LEN_INDEX] = TOTAL_SENSORS_PACKET_LEN;
    u8TxBuffer[FUNCTION_CODE_INDEX] = SENSORS_DATA_FC;
    u8TxBuffer[NUMBER_ELEMENTS_INDEX] = TOTAL_NUMBER_OF_SENSORS;
    for (int i = 0; i < TOTAL_NUMBER_OF_SENSORS; i++)
    {
      u8TxBuffer[DATA_PAYLOAD_INDEX + i * 2] = (unsigned char)(((sensorOutputData[i].unitX10 & 0xFF00) >> 8) & 0x00FF);
      u8TxBuffer[DATA_PAYLOAD_INDEX + i * 2 + 1] = (unsigned char)(sensorOutputData[i].unitX10 & 0x00FF);
    }
    sendDataLen = TOTAL_SENSORS_PACKET_LEN;
  }
  else if (ePacketType == PARAMS_DATA)
  {
    u8TxBuffer[PACKET_LEN_INDEX] = TOTAL_PARAMS_PACKET_LEN;
    u8TxBuffer[FUNCTION_CODE_INDEX] = PARAMETERS_DATA_FC;
    u8TxBuffer[NUMBER_ELEMENTS_INDEX] = TOTAL_NUMBER_OF_PARAMS;
    for (int i = 0; i < TOTAL_NUMBER_OF_PARAMS; i++)
    {
      u8TxBuffer[DATA_PAYLOAD_INDEX + i * 2] = (unsigned char)(((params[i].value_curr_mem & 0xFF00) >> 8) & 0x00FF);
      u8TxBuffer[DATA_PAYLOAD_INDEX + i * 2 + 1] = (unsigned char)(params[i].value_curr_mem & 0x00FF);
    }
    sendDataLen = TOTAL_PARAMS_PACKET_LEN;
  }
  else
  {
    sendDataLen = 0;
    /*packet type unknown .. ignore it*/
  }

  if (sendDataLen != 0)
  {
    crc16Val = crc16(u8TxBuffer, u8TxBuffer[PACKET_LEN_INDEX] - 2);
    u8TxBuffer[SENSORS_CRC_2B_INDEX] = (unsigned char)(((crc16Val & 0xFF00) >> 8) & 0x00FF);
    u8TxBuffer[SENSORS_CRC_2B_INDEX + 1] = (unsigned char)(crc16Val & 0x00FF);
    Serial3.write(u8TxBuffer, sendDataLen);
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
    Serial.println(rxdata);
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
                Serial.print(command);
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
        if (sensorOutputData[PS1].unitX10 > (params[PEAK_PRES].value_curr_mem * 10) )
        {
          if (bSendPeakHighDetected == false)
          {
            bSendPeakHighDetected = true;
            //Serial2.print(commands[INH_SOLE_OFF]);
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
            // Serial2.print(commands[EXH_SOLE_OFF]);
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


void abc() {
  /*Index_select loop*/
  //  unsigned long lastRotateTime = 0;
  //  unsigned long rotateTime = millis();
  //  currentCLK = digitalRead(DISP_ENC_CLK);
  //  while (READ_FROM_ENCODER) {
  //    currentCLK = digitalRead(DISP_ENC_CLK);
  //    int incr = 0;
  //    if (currentCLK != lastCLK) {
  //      if (currentCLK != lastCLK && currentCLK == 1) {
  //        if (digitalRead(DISP_ENC_DT) != currentCLK ) {
  //          incr--;
  //        } else {
  //          incr++;
  //        }
  //      }
  //      resetEditModetime = millis();
  //    }
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
        ;lcd.setCursor(VALUE1_DISPLAY_POS, 1);
        lcd.setCursor(3,1);
        lcd.print("New IER  1:");
        lcd.print(newIER);
        lcd.setCursor(3,2);
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
        lcd.setCursor(0,3);
        lcd.print("set using PEEP valve");
      }
      incr=0;
    }
    lastCLK = currentCLK;
  }while ((millis() - resetEditModetime) < EDIT_MODE_TIMEOUT);
}
