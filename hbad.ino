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
    lcd.setCursor(0,0);
    lcd.print("RunTime parameters");
    runInitDisplay = false;
  }
  cleanRow(1);cleanRow(2);cleanRow(3);

  lcd.setCursor(0,1);
  lcd.print("TV:");
  lcd.print(tidl_volu.value_curr_mem);

  lcd.setCursor(7,1);
  lcd.print("RR:");
  lcd.print(resp_rate.value_curr_mem);

  lcd.setCursor(13,1);
  lcd.print("IER:");
  lcd.print(inex_rati.value_curr_mem);


  lcd.setCursor(0,2);
  lcd.print("PP:");
  lcd.print(peep_pres.value_curr_mem);
  
  lcd.setCursor(6,2);
  lcd.print("O2:");
  lcd.print((PS_ReadSensorValueX10(O2))/10);

  lcd.setCursor(12,2);
  lcd.print("IP:");
  lcd.print(PS_ReadSensorValueX10(PS1)/10);

  lcd.setCursor(0,3);
  lcd.print("EP:");
  lcd.print(PS_ReadSensorValueX10(PS2)/10);
}


void loop() {
  RT_Events_T eRTState;
  displayRunTime();
  eRTState = encoderScanUnblocked();
  if (eRTState == RT_BT_PRESS)
  {
    Serial.println("Entering Edit mode!");
    MsTimer2::stop();
    editMode();
    MsTimer2::start();
    runInitDisplay = true;
  }
  delay (50);
  if(gSensorDataUpdated ==1)
  {
    gSensorDataUpdated = 0;
    UART3_SendDAQDataGraphicDisplay(SENSORS_DATA);
  }
  if(gCtrlParamUpdated == 1)
  {
    gCtrlParamUpdated = 0;
    UART3_SendDAQDataGraphicDisplay(PARAMS_DATA);
  }
}

void editMode()
{
  lcd.clear();
  switchMode = DISPLAY_MODE;
  resetEditModetime=millis();
  do{
    //  sendCommands();
    announce();
    delay(100);
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
  }while ((millis() - resetEditModetime) < 5000);
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
    int storedValue = getCalibValue(params[currPos].value_curr_mem, currPos);
    if(storedValue<100){
      lcd.print(" ");
    }
    lcd.print(storedValue);
    lcd.setCursor(14, 3);
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
      storeParam(params[currPos]);
      params[currPos].value_curr_mem = params[currPos].value_new_pot;
      //    Serial.println(params[currPos].value_curr_mem);
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
 // #if SERIAL_PRINTS
  Serial.println("we are in diagO2Sensor");
 // #endif
  
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
}
void diagSolStatus(void)
{
    Serial.println("we are in diagSolStatus");
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
