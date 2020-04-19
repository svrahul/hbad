#include "Variables.h"

/*
 * Sensor values index
 */

#define READ_ALL_CTRL_PARAMS  MAX_CTRL_PARAMS
#define READ_ALL_SENSORS_DATA   NUM_OF_SENSORS


/* Packet received timeout */
#define MAX_BYTE_SILENCE_TIME 100

#define PACKET_HEAD_LEN 3

#define ACTION_READ   0x01
#define ACTION_WRITE  0x02

#define SOLENOID_CLOSE   0x01
#define SOLENOID_OPEN  0x00

#define START 0x00
#define STOP  0x01

#define EDITMODE_ENTRY  0x01
#define EDITMODE_EXIT   0x02

#define DEST_ADDR_INDEX   0
#define FUNCTION_CODE_INDEX 1
#define DATA_PAYLOAD_LEN_INDEX  2
#define DATA_PAYLOAD_INDEX 3
#define NUMBER_ELEMENTS_INDEX DATA_PAYLOAD_INDEX


/*Device address */
#define MOTOR_CONTROLLER_ADDR 0x01
#define DATA_AQUISITION_ADDR  0x02
#define GRAPHICS_DISPLAY_ADDR 0x03

/*DAQ module function codes at index 2*/
/*typedef enum
{
  READ_SENSORS_DATA_FC = 0x01,
  READ_CTRL_PARAMS_FC,
  SET_CTRL_STATE_FC
}UartPacketFCDef;
*/
/*==========  Functions code ===========*/
#define READ_SENSORS_DATA_FC    0x01
#define READ_CTRL_PARAMS_FC     0x02
#define SET_CTRL_STATE_FC       0x03
/*Control Functions */
#define OXY_CYLIN_SOLENOID_FC   0x04
#define OXY_HOPS_O2_SOLENOID_FC 0x05
#define INHALE_SOLENOID_FC      0x06
#define EXHALE_SOLENOID_FC      0x07
#define PP_RELIEF_SOLENOID      0x08
/*Stepper motor control control FC*/
#define STEPPER_MOTOR_CTRL_FC   0x09
/*Control Modules Functions */
#define INIT_MASTER_FC          0x0A
#define INIT_VALVE_BLOCK_FC     0x0B
#define INIT_STEPPER_MODULE_FC  0x0C
#define INIT_BREATH_DETECT      0x0D

#define USER_EDIT_MODE          0x20
/*=====================================*/

#define CHECK_CONTROL_FC(x)     (((x>=OXY_CYLIN_SOLENOID_FC)&&(x<=STEPPER_MOTOR_CTRL_FC))?true:false)
#define CHECK_INITMODULE_FC(x)  (((x>=INIT_MASTER_FC)&&(x<=INIT_BREATH_DETECT))?true:false)
#define CHECK_STEPPER_CONTROL_FC(x)  ((x==STEPPER_MOTOR_CTRL_FC)?true:false)

#define SET_CTRL_STATE_CMD_LEN  0x06

/* UART2 buffer max size */
#define MAX_UART2_BUFF_SIZE 0x20
#define MAX_RX_BUFFERS 2
unsigned char RXBuff[MAX_RX_BUFFERS][MAX_UART2_BUFF_SIZE];
int ActiveBuffIndex =0;
unsigned char *UartRxBuff = RXBuff[ActiveBuffIndex];
unsigned char *PacketBuff = RXBuff[ActiveBuffIndex];
int UARTIndex = 0;
int PacketLen = 0;

unsigned long int Uart2Msec =0;
#define MAX_TX_LEN  25
/* Global TX Buffer for sending the data over UART3 to Graphics display  module 
* and UART2 to Motor control module 
*/
unsigned char u8TxBuf[MAX_TX_LEN];


bool CRCVerify(unsigned char *LenCtrlEventBuff, int PacketLen);
bool UART2_IsCtrlPacketRecevied(void);
bool CRCVerify(unsigned char *LenCtrlEventBuff, int PacketLen);

void Process_Sensors_Data(void);
void Process_Contrl_Parameters(void);
void Process_UserEditStatus(void);
void Process_Init_Modules(void);
void Send_SystemState(ControlStatesDef_T eState);

void Ctrl_ProcessRxData(void) 
{
  int index =0;
  if((PacketBuff[DEST_ADDR_INDEX] == MOTOR_CONTROLLER_ADDR) && \
      (CRCVerify(PacketBuff, PacketLen) == true))
  {
    switch(PacketBuff[FUNCTION_CODE_INDEX])
    {
      case READ_SENSORS_DATA_FC:
      {
        Process_Sensors_Data();   
      }
      break;
      case READ_CTRL_PARAMS_FC:
      {
        Process_Contrl_Parameters();
      }
      break;
      case OXY_CYLIN_SOLENOID_FC:
      if(PacketBuff[DATA_PAYLOAD_INDEX] == ACTION_WRITE)
      {
        if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_CLOSE)
        {
          O2Cyl_VLV_CLOSE();
        }
        else if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_OPEN)
        {
          O2Cyl_VLV_OPEN();
        }
        else
        {
          
        }
      }
      break;   
      case OXY_HOPS_O2_SOLENOID_FC:
      {
        if(PacketBuff[DATA_PAYLOAD_INDEX] == ACTION_WRITE)
        {
          if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_CLOSE)
          {
            O2Hln_VLV_CLOSE();
          }
          else if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_OPEN)
          {
            O2Hln_VLV_OPEN();
          }
          else
          {
            
          }
        }
      }
      break;
      case INHALE_SOLENOID_FC:
      {
        if(PacketBuff[DATA_PAYLOAD_INDEX] == ACTION_WRITE)
        {
          if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_CLOSE)
          {
            INHALE_VLV_CLOSE(); 
            //Stop motor
            if ((cycle_start == true) && (comp_start == true) && (comp_end == false)) 
            {
              Emergency_motor_stop = true;
            }
            //relief valve ON
            INHALE_RELEASE_VLV_OPEN();
          }
          else if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_OPEN)
          {
            INHALE_VLV_OPEN();
          }
          else
          {
            
          }
        }
      }
      case EXHALE_SOLENOID_FC:
      {
        if(PacketBuff[DATA_PAYLOAD_INDEX] == ACTION_WRITE)
        {
          if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_CLOSE)
          {
            EXHALE_VLV_CLOSE();
          }
          else if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_OPEN)
          {
            EXHALE_VLV_OPEN();
          }
          else
          {
            
          }
        }
      }
      case PP_RELIEF_SOLENOID:
      {
        if(PacketBuff[DATA_PAYLOAD_INDEX] == ACTION_WRITE)
        {
          if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_CLOSE)
          {
            INHALE_RELEASE_VLV_CLOSE();
          }
          else if(PacketBuff[DATA_PAYLOAD_INDEX+1] == SOLENOID_OPEN)
          {
            INHALE_RELEASE_VLV_OPEN();
          }
          else
          {
            
          }
        }
      }
      case STEPPER_MOTOR_CTRL_FC:
      {
        if(PacketBuff[DATA_PAYLOAD_INDEX] == ACTION_WRITE)
        {
          if(PacketBuff[DATA_PAYLOAD_INDEX+1] == START)
          {
            int_start();
          }
          else
          {
            int_stop();
          }
        }  
      }
      break;
      case INIT_MASTER_FC:
      {
      int_syst();
      int_stop();
      }
      case INIT_VALVE_BLOCK_FC:
      {
        int_valves();
      }
      break;
      case INIT_STEPPER_MODULE_FC:
      {
        /*Not implemented*/
      }
      break;
      case INIT_BREATH_DETECT:
      {
        /* Yet to be implemented and TBD*/
      }
      break;
      case USER_EDIT_MODE:
      {
        /* Assisted mode yet to be discussed */
      }
      break;
      default: 
      {
        /*if(PacketBuff[FUNCTION_CODE_INDEX] == 0x04)
        {
          Set_SolenoidOnOff(PacketBuff[3],PacketBuff[4]);
        }
        else if(PacketBuff[FUNCTION_CODE_INDEX] == 0x05)
        {
          Init_ControllerModule(PacketBuff[3]);
        }
        else
        {
          
        }*/        
      }
      break;
    }    
  }
}
/*
 * This function forms the packet includes new systems state and sends on UART   
*/
void Send_SystemState(ControlStatesDef_T eState)
{
  int index=0;
  u8TxBuf[index++] = DATA_AQUISITION_ADDR; //Destination address
  u8TxBuf[index++] = SET_CTRL_STATE_FC;   // Function code
  u8TxBuf[index++] = 1; //Data Payload length
  u8TxBuf[index++] = eState;     
  UART2_SendDatawithCRC16(u8TxBuf, index);
}

/*
 * This function forms the packet includes new systems state and sends on UART   
*/
void Read_ControlParams(ControlParams_T eCtrlParam)
{
  int index=0;
  u8TxBuf[index++] = DATA_AQUISITION_ADDR; //Destination address
  u8TxBuf[index++] = READ_CTRL_PARAMS_FC;   // Function code
  u8TxBuf[index++] = 1; //Data Payload length
  u8TxBuf[index++] = eCtrlParam;     
  UART2_SendDatawithCRC16(u8TxBuf, index);
}

/*
 * This function forms the packet includes new systems state and sends on UART   
*/
void Read_SensorsData(sensor_e eSensor)
{
  int index=0;
  u8TxBuf[index++] = DATA_AQUISITION_ADDR; //Destination address
  u8TxBuf[index++] = READ_SENSORS_DATA_FC;   // Function code
  u8TxBuf[index++] = 1; //Data Payload length
  u8TxBuf[index++] = eSensor;     
  UART2_SendDatawithCRC16(u8TxBuf, index);
}

/*
 * Call back function to parse the packet corresponding to   
 * updates that whether user entered/exited edit mode on user interface
*/
void Process_UserEditStatus(void)
{
  
}

/*
 * Call back function to parse the packet corresponding to   
 * different initialization modules 
*/
void Process_Init_Modules(void)
{
  
}
/*
 * Call back function to parse the packet to decode sensors values 
*/

void Process_Sensors_Data(void)
{
  int index =0;
  if(PacketBuff[NUMBER_ELEMENTS_INDEX] <= NUM_OF_SENSORS)
  {
    if(PacketBuff[NUMBER_ELEMENTS_INDEX] == READ_ALL_SENSORS_DATA)
    {
      for(int i=0; i<NUM_OF_SENSORS; i++)
      {
        index = PacketBuff[DATA_PAYLOAD_INDEX+i*3];
        if(index < NUM_OF_SENSORS)
        {
          sensorOutputData[index].unitX10 = (PacketBuff[index+1]<<8) | PacketBuff[index+2];
        }        
      }      
    }
    else
    {
        index = PacketBuff[DATA_PAYLOAD_INDEX];
        if(index < NUM_OF_SENSORS)
        {
          sensorOutputData[index].unitX10 = (PacketBuff[index+1]<<8) | PacketBuff[index+2];
        } 
    }
  }
}
/*
 * Call back function to parse control parameters from comm packet
*/
void Process_Contrl_Parameters(void)
{
  int index = 0;
  if(PacketBuff[NUMBER_ELEMENTS_INDEX] <= MAX_CTRL_PARAMS)
  {
    if(PacketBuff[NUMBER_ELEMENTS_INDEX] == READ_ALL_CTRL_PARAMS)
    {
      for(int i=0; i < MAX_CTRL_PARAMS; i++)
      {
        index = PacketBuff[DATA_PAYLOAD_INDEX+i*3];
        if(index < MAX_CTRL_PARAMS)
        {
          CtrlParams[index] = (PacketBuff[index+1]<<8) | PacketBuff[index+2];
          Update_LocalControlParamVars(index);
        }        
      }      
    }
    else
    {
        index = PacketBuff[DATA_PAYLOAD_INDEX];
        if(index < MAX_CTRL_PARAMS)
        {
          CtrlParams[index] = (PacketBuff[index+1]<<8) | PacketBuff[index+2];
          Update_LocalControlParamVars(index);
        } 
    }
    
  }
}
void Update_LocalControlParamVars(int CtrlIndex)
{
  switch(CtrlIndex)
  {
    case TIDAL_VOL: 
    {
      tidal_volume_new = CtrlParams[CtrlIndex]; 
      Set_NewStrokeLength(tidal_volume_new);
    }
    break;
    case BREATHS_PM: BPM_new = CtrlParams[CtrlIndex];  break;
    case PEAK_PRES: peak_prsur = CtrlParams[CtrlIndex]; break;
    case FIO2_PERC: FiO2 = CtrlParams[CtrlIndex]; break;
    case IE_RATIO: IER_new = CtrlParams[CtrlIndex]; break;
    case PEEP_PRES: PEEP_new = CtrlParams[CtrlIndex]; break;
    default: break;
  }
}
/*
 * Function to adjust the new stroke length based on the new Tidal volume setting 
 */
void Set_NewStrokeLength(int tidal_volume_new)
{
  if (tidal_volume_new == 50) Stroke_length_new = 16;
  if (tidal_volume_new == 100) Stroke_length_new = 24;
  if (tidal_volume_new == 150) Stroke_length_new = 31;
  if (tidal_volume_new == 200) Stroke_length_new = 37;
  if (tidal_volume_new == 250) Stroke_length_new = 42;
  if (tidal_volume_new == 300) Stroke_length_new = 48;
  if (tidal_volume_new == 350) Stroke_length_new = 53;
  if (tidal_volume_new == 400) Stroke_length_new = 57;
  if (tidal_volume_new == 450) Stroke_length_new = 62;
  if (tidal_volume_new == 500) Stroke_length_new = 66;
  if (tidal_volume_new == 550) Stroke_length_new = 70;
  if (tidal_volume_new == 600) Stroke_length_new = 74;
  if (tidal_volume_new == 650) Stroke_length_new = 78;
  if (tidal_volume_new == 700) Stroke_length_new = 82;
  if (tidal_volume_new == 750) Stroke_length_new = 86;
  if (tidal_volume_new == 800) Stroke_length_new = 90;
}
/*
 * Serial byte received ISR for UART2
 */
void serialEvent2() 
{
  Uart2Msec = millis();
  while (Serial2.available()) 
  { 
    if(UARTIndex < MAX_UART2_BUFF_SIZE)
    {
      UartRxBuff[UARTIndex] = Serial2.read();
      //Serial.println(UartRxBuff[UARTIndex]);
      UARTIndex++;
    }
  }
}
/*
 * This function called from loop to periodically check if there is any 
 * packet received from Motor control module. returns true, if available and
 * also copies the buffer and packet length to be transmitted 
 */
bool UART2_IsCtrlPacketRecevied(void)
{
  unsigned long int CurrentMSec;
  bool StatusF = false;
  if(Uart2Msec)
  {
      CurrentMSec = millis();     
      if((CurrentMSec - Uart2Msec) > MAX_BYTE_SILENCE_TIME)
      {        
        Uart2Msec = 0;
        StatusF = true;
        PacketBuff = UartRxBuff;   
        PacketLen = UARTIndex; 
        ActiveBuffIndex++;
        if(ActiveBuffIndex >= MAX_RX_BUFFERS)
        {
          ActiveBuffIndex =0;      
        }
        UartRxBuff = RXBuff[ActiveBuffIndex];
        UARTIndex=0;      
      }
  }
  else
  {
    /* No packet available to process */
  }
  return(StatusF);
}
/*
 * This functtion calculates the crc16, appends at the end before sending on UART
 */
void UART2_SendDatawithCRC16(unsigned char *Buffer, int len)
{
  int crcVal = crc16(Buffer, len);
  u8TxBuf[len++]= ((crcVal>>8) & 0xFF);
  u8TxBuf[len++]= (crcVal & 0xFF);      
  Serial2.write(u8TxBuf, len);
  /*Serial.println("Event Data Len= "+ String(index));
  for(int i=0; i<index; i++)
  {
    Serial.print(" " + String(u8TxBuf[i]));    
  }
  */
}
/*
 * Function to verify the integrity of packet received from Motor control.
 */
bool CRCVerify(unsigned char *RxPacket, int PacketLen)
{
  int crc16Received;
  int crc16Measured;

  crc16Received = ((RxPacket[PacketLen-2]<<8) | RxPacket[PacketLen-1]);
  crc16Measured = crc16(RxPacket, PacketLen-2); 
  return(crc16Received == crc16Measured);
}
 
#define POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/

unsigned short crc16(unsigned char *data_p, unsigned short length)
{
  unsigned char i;
  unsigned int data;
  unsigned int crc = 0xffff;
  
  if (length == 0)
        return (~crc);  
  do
  {
    for (i=0, data=(unsigned int)0xff & *data_p++;
         i < 8; 
         i++, data >>= 1)
    {
      if ((crc & 0x0001) ^ (data & 0x0001))
            crc = (crc >> 1) ^ POLY;
      else  crc >>= 1;
    }
  } while (--length);
  
  crc = ~crc;
  data = crc;
  crc = (crc << 8) | (data >> 8 & 0xff);
  
  return (crc);
}


/*

void Prcs_RxData() {
  String p1;
  String p2;
  String p3;
  String p4;
  String payload;

  p1 = rxdata.substring(1, 3);
  p2 = rxdata.substring(3, 5);
  p3 = rxdata.substring(5, 7);
  p4 = rxdata.substring(7, 9);
  payload = p3 + p4;


  if (p1 == "VM") 
  {
    if (p2 == "ST") 
    {
      //stepper motor
      if (payload == "0000") {
        int_stop();
      }
    } 
    else if (p2 == "IN") 
    {
      if (payload == "0000") {
        if (cycle_start == false) int_syst();
      }
      if (payload == "0001") {
        int_stop();
      }
      if (payload == "0002") {
        int_start();
      }
      if (payload == "0003") {
        int_valves();
      }
    } 
    else if (p2 == "PP") 
    {
      if (payload == "0000") {
        Par_editstat = 1;
      }
      if (payload == "1111") {
        Par_editstat = 0;
      }
    }
    else if (p2 == "P1") 
    {
      tidal_volume_new = payload.toInt();
      Serial.print("TV : "); Serial.println(tidal_volume_new);
      //Stroke_length_new = 3.094451 - (0.03629902 * tidal_volume_new) + (0.06564267 * tidal_volume_new * tidal_volume_new);
      //y = 3.094451 - 0.03629902*x + 0.06564267*x^2
      //Stroke_length_new = 12.0522 + (0.251663 * tidal_volume_new) - (0.0001995534 * tidal_volume_new * tidal_volume_new);
      //y = 12.0522 + 0.251663*x - 0.0001995534*x^2
      //big head pusher
      //Stroke_length_new = 3499446 + (2.008683 - 3499446) / (1 + ((tidal_volume_new / 7124337000) ^ 0.6621993));
      //4pl//y = 3499446 + (2.008683 - 3499446)/(1 + (x/7124337000)^0.6621993)
      //big head pusher
      //Stroke_length_new = 6.352569 + (0.1893826 * tidal_volume_new) - (0.0002018492 * (tidal_volume_new * tidal_volume_new)) + (1.215626e-7 * (tidal_volume_new * tidal_volume_new * tidal_volume_new));
      //y = 6.352569 + 0.1893826*x - 0.0002018492*x^2 + 1.215626e-7*x^3
      //Stroke_length_new = 3926850 + ((2.261358 - 3926850)/(1 + pow((tidal_volume_new/7122523000),0.6693673)));

   
      //4PL
      //x=vt & y=StrokeLength
      //y = 3926850 + (2.261358 - 3926850)/(1 + (x/7122523000)^0.6693673)
      //x=SL & y=VT
      //y = 4707.314 + (1.557452 - 4707.314)/(1 + (x/225.8712)^1.729569)


      if (tidal_volume_new == 50) Stroke_length_new = 16;
      if (tidal_volume_new == 100) Stroke_length_new = 24;
      if (tidal_volume_new == 150) Stroke_length_new = 31;
      if (tidal_volume_new == 200) Stroke_length_new = 37;
      if (tidal_volume_new == 250) Stroke_length_new = 42;
      if (tidal_volume_new == 300) Stroke_length_new = 48;
      if (tidal_volume_new == 350) Stroke_length_new = 53;
      if (tidal_volume_new == 400) Stroke_length_new = 57;
      if (tidal_volume_new == 450) Stroke_length_new = 62;
      if (tidal_volume_new == 500) Stroke_length_new = 66;
      if (tidal_volume_new == 550) Stroke_length_new = 70;
      if (tidal_volume_new == 600) Stroke_length_new = 74;
      if (tidal_volume_new == 650) Stroke_length_new = 78;
      if (tidal_volume_new == 700) Stroke_length_new = 82;
      if (tidal_volume_new == 750) Stroke_length_new = 86;
      if (tidal_volume_new == 800) Stroke_length_new = 90;

      Serial.print("SL : "); Serial.println(Stroke_length_new);
      Serial2.print("$VSP20001&");
    }
    else if (p2 == "P2") 
    {
      BPM_new = payload.toInt();
      Serial.print("BPM : "); Serial.println(BPM_new);
      Serial2.print("$VSP50004&");
    }
    else if (p2 == "P3") 
    {
      peak_prsur = payload.toInt();
      //Serial.print("Peak_prsur : "); Serial.println(peak_prsur);
    }
    else if (p2 == "P4") 
    {
      FiO2 = payload.toInt();
      //Serial.print("FiO2 : "); Serial.println(FiO2);
    }
    else if (p2 == "P5") 
    {
      IER_new = payload.toInt();
      //Serial.print("IER : "); Serial.println(IER_new);
      //      IER = 1020;
      //      inhale_ratio = 1.0;
      //      exhale_ratio = 2.0;
    }
    else if (p2 == "P6") 
    {
      PEEP_new = payload.toInt();
      //Serial.print("PEEP_new : "); Serial.println(PEEP_new);
    }
    else if (p2 == "SV") 
    {
      if (p3 == "01") {
        if (p4 == "00") {
          //digitalWrite(INHALE_VLV_PIN, LOW);
          INHALE_VLV_CLOSE(); 
          //Stop motor
          if ((cycle_start == true) && (comp_start == true) && (comp_end == false)) Emergency_motor_stop = true;
          //relief valve ON
          INHALE_RELEASE_VLV_OPEN();
        } else if (p4 == "01") {
          //digitalWrite(INHALE_VLV_PIN, HIGH);
          INHALE_VLV_OPEN();
        }
      } else if (p3 == "02") {
        if (p4 == "00") {
          //digitalWrite(EXHALE_VLV_PIN, LOW);
          INHALE_VLV_CLOSE();
        } else if (p4 == "01") {
          //digitalWrite(EXHALE_VLV_PIN, HIGH);
          INHALE_VLV_OPEN();
        }
      }
    } else if (p2 == "O2") {   //solanoide valve for Oxygen line
      if (p3 == "01") {
        if (p4 == "00") {
          //digitalWrite(O2Cyl_VLV_PIN, LOW);
          O2Cyl_VLV_CLOSE(); 
        } else if (p4 == "01") {
          //digitalWrite(O2Cyl_VLV_PIN, HIGH);
          O2Cyl_VLV_OPEN();
        }
      } else if (p3 == "02") {
        if (p4 == "00") {
          //digitalWrite(O2Hln_VLV_PIN, LOW);
          O2Hln_VLV_CLOSE();
        } else if (p4 == "01") {
          //digitalWrite(O2Hln_VLV_PIN, HIGH);
          O2Hln_VLV_OPEN();
        }
      }
    }
  }
}
*/


void int_valves() {
  //Normally Opened
  EXHALE_VLV_OPEN();
  INHALE_VLV_OPEN();

  //Normally closed
  INHALE_RELEASE_VLV_CLOSE();
  O2Cyl_VLV_CLOSE();
  O2Hln_VLV_CLOSE();
}

void int_start() {
  Emergency_motor_stop = false;
  Serial.println("Skipping Home Cycle : ");
  home_cycle = false;
  cycle_start = true;
  comp_start = false;
  comp_end = false;
  exp_start = true;
  exp_end = true;
  //run_motor = true;
}

void int_stop() {
  Emergency_motor_stop = false;
  run_motor = true;
  Serial.println("Cycle Stop & goto Home : ");
  run_pulse_count = 200000;
  digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
  live_calculate_value(home_speed_value * 10.0);
  comp_start = false;
  comp_end = false;
  exp_start = false;
  exp_end = false;
  home_cycle = true;
  cycle_start = false;
  run_motor = true;
}

void int_syst() {
  Emergency_motor_stop = false;
  motion_profile_count_temp = 0;
  run_pulse_count_temp = 0.0;
  if (digitalRead(HOME_SENSOR_PIN) == 1)
  {
    Serial.println("Home Cycle : ");
    run_pulse_count = 200000;
    digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
    live_calculate_value(home_speed_value * 10.0);
    comp_start = false;
    comp_end = false;
    exp_start = false;
    exp_end = false;
    home_cycle = true;
    run_motor = true;
    delay(200);
    cycle_start = true;
    Serial2.print("$VSSY0000&");
  } else {
    Serial.println("Skipping Home Cycle : ");
    calculate_speed_position();
    home_cycle = false;
    comp_start = false;
    comp_end = false;
    exp_start = true;
    exp_end = true;
    //run_motor = true;
    cycle_start = true;
  }
}
