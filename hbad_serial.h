

typedef enum
{
  SENSORS_DATA=0,
  PARAMS_DATA,
}UartPacketTypeDef;

#define MAX_TX_LEN  25
/* Global TX Buffer for sending the data over UART3 to Graphics display  module 
* and UART2 to Motor control module 
*/
unsigned char u8TxBuf[MAX_TX_LEN];


/*Packet definition for sending sensors data to graphics display module */

#define TOTAL_NUMBER_OF_SENSORS NUM_OF_SENSORS
#define TOTAL_NUMBER_OF_PARAMS  MAX_CTRL_PARAMS



#define DEST_ADDR_INDEX   0
#define FUNCTION_CODE_INDEX 1
#define DATA_PAYLOAD_LEN_INDEX  2
#define DATA_PAYLOAD_INDEX 3
#define NUMBER_ELEMENTS_INDEX DATA_PAYLOAD_INDEX

#define SENSORS_CRC_2B_INDEX  (DATA_PAYLOAD_INDEX + 1 + TOTAL_NUMBER_OF_SENSORS*2) 
#define PARAMS_CRC_2B_INDEX  (DATA_PAYLOAD_INDEX + 1 + TOTAL_NUMBER_OF_PARAMS*2)

#define TOTAL_SENSORS_PACKET_LEN  (SENSORS_CRC_2B_INDEX+2)
#define TOTAL_PARAMS_PACKET_LEN (PARAMS_CRC_2B_INDEX+2)

#if (TOTAL_SENSORS_PACKET_LEN > TOTAL_PARAMS_PACKET_LEN)
#define TOTAL_PACKET_LEN TOTAL_SENSORS_PACKET_LEN
#else
#define TOTAL_PACKET_LEN TOTAL_PARAMS_PACKET_LEN
#endif

/*
 * Communication packet definition
 * <1B Dest Addr><1B Function code><1B Payload len><Pay load><2B CRC>
 */

/* Packet received timeout */
#define MAX_BYTE_SILENCE_TIME 100

#define PACKET_HEAD_LEN 3

#define ACTION_READ   0x01
#define ACTION_WRITE  0x02

#define SOLENOID_ON   0x01
#define SOLENOID_OFF  0x00

#define EDITMODE_ENTRY  0x01
#define EDITMODE_EXIT   0x02

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


/*Read sensors data FC sub function code */
#define READ_SENSORS_CMD_LEN  6
#define READ_ALL_SENSORS_DATA   NUM_OF_SENSORS
#define READ_SENSOR1_DATA 0x01
#define READ_SENSOR2_DATA 0x02
#define READ_SENSOR3_DATA 0x03
#define READ_SENSOR4_DATA 0x04
#define READ_SENSOR5_DATA 0x05
#define READ_SENSOR6_DATA 0x06


/*Read parameters data FC sub function codes */
#define READ_CTRL_PARAMS_CMD_LEN  6
#define READ_ALL_CTRL_PARAMS  MAX_CTRL_PARAMS
#define READ_CTRL_PARAM1  0x01
#define READ_CTRL_PARAM2  0x02
#define READ_CTRL_PARAM3  0x03
#define READ_CTRL_PARAM4  0x04
#define READ_CTRL_PARAM5  0x05
#define READ_CTRL_PARAM6  0x06

/* Function declarations */
void UART3_SendDAQDataGraphicDisplay(UartPacketTypeDef ePacketType);
unsigned short crc16(unsigned char *data_p, unsigned short length);
bool UART2_IsCtrlPacketRecevied(unsigned char **Buff,  int *Len);
bool CRCVerify(unsigned char *LenCtrlEventBuff, int EvetDataLen);
void timerIsr(void);
void UART2RxIsr(void);
void UART2_SendDatawithCRC16(unsigned char *Buffer, int len);

void Send_Contrl_Parameters(int param_index);
void Send_Sensors_Data(int sensor_index);
void Init_ControllerModule(int InitModule);

void Set_StepperMotorOnOff(int OnOff);
void Set_SolenoidOnOff(int SolenoidIndex, int OnOff);

void Read_SolenoidStatus(int SolenoidIndex);
void Read_StepperMotorStatus(void);

void Send_UserEditMode(int EntryExit);

/* UART2 buffer max size */
#define MAX_UART2_BUFF_SIZE 0x20
#define MAX_RX_BUFFERS 2
unsigned char RXBuff[MAX_RX_BUFFERS][MAX_UART2_BUFF_SIZE];
int ActiveBuffIndex =0;
int LastByteReceivedTime = 0;
unsigned char *UartRxBuff = RXBuff[ActiveBuffIndex];
unsigned char *PacketBuff = RXBuff[ActiveBuffIndex];
int UARTIndex = 0;
int PacketLen = 0;

unsigned long int Uart2Msec =0;

/*
 * This function forms the command packet to initialize the different modules of MC 
 * and sends it on the UART2
 */
void Init_ControllerModule(int InitModule)
{
  int index=0;
  if(CHECK_INITMODULE_FC(InitModule))
  {
    u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
    u8TxBuf[index++] = InitModule;   // Function code
    u8TxBuf[index++] = 0x00;
    UART2_SendDatawithCRC16(u8TxBuf, index);
  }
}

/*
 * This function forms the command packets to control solenoids and send it on the UART2
 */
void Set_SolenoidOnOff(int SolenoidIndex, int OnOff)
{
  int index=0;
  if(CHECK_CONTROL_FC(SolenoidIndex))
  {
    u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
    u8TxBuf[index++] = SolenoidIndex;   // Function code
    u8TxBuf[index++] = 2; //Data Payload length
    u8TxBuf[index++] = ACTION_WRITE;
    if(OnOff)
    {
       u8TxBuf[index++] = 1;  
    }
    else
    {
      u8TxBuf[index++] = 0;
    }
    UART2_SendDatawithCRC16(u8TxBuf, index);
  }
}
/*
 * This function forms the command packet to read a particular solenoid 
 * and sends it on the UART2
 */
void Read_SolenoidStatus(int SolenoidIndex)
{
  int index=0;
  if(CHECK_CONTROL_FC(SolenoidIndex))
  {
    u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
    u8TxBuf[index++] = SolenoidIndex;   // Function code
    u8TxBuf[index++] = 1; //Data Payload length
    u8TxBuf[index++] = ACTION_READ;   
    UART2_SendDatawithCRC16(u8TxBuf, index);
  }
}

/*
 * This function forms the command packet to On Off the stepper motor 
 * and sends it on the UART2
 */
void Set_StepperMotorOnOff(int OnOff)
{
  int index=0;
  u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
  u8TxBuf[index++] = STEPPER_MOTOR_CTRL_FC;   // Function code
  u8TxBuf[index++] = 2; //Data Payload length
  u8TxBuf[index++] = ACTION_WRITE;   
  if(OnOff)
  {
    u8TxBuf[index++] = 1;
  }
  else
  {
    u8TxBuf[index++] = 0;
  }
  UART2_SendDatawithCRC16(u8TxBuf, index);
}
/*
 * This function forms the command packet to read Stepper motor status 
 * and sends it on the UART2
 */
void Read_StepperMotorStatus(void)
{
  int index=0;
  
  u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
  u8TxBuf[index++] = STEPPER_MOTOR_CTRL_FC;   // Function code
  u8TxBuf[index++] = 1; //Data Payload length
  u8TxBuf[index++] = ACTION_READ;   
  UART2_SendDatawithCRC16(u8TxBuf, index);
}

/*
 * This function forms the command packet to let know MC about 
 * user edit mode entry and exit and sends it on the UART2
 */
void Send_UserEditMode(int EntryExit)
{
  int index=0;
  u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
  u8TxBuf[index++] = USER_EDIT_MODE;   // Function code
  u8TxBuf[index++] = 0x01;
  if(EntryExit)
  {
    u8TxBuf[index++] = EDITMODE_ENTRY;
  }
  else
  {
    u8TxBuf[index++] = EDITMODE_EXIT;
  }
  UART2_SendDatawithCRC16(u8TxBuf, index);
}
/*
 * This function forms the response packet with control parameters and send it on the UART2
 */
void Send_Contrl_Parameters(int param_index)
{
  int index =0;
  u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR; //Destination address
  u8TxBuf[index++] = READ_CTRL_PARAMS_FC;   // Function code   
  if(param_index == READ_ALL_CTRL_PARAMS) 
  {
    u8TxBuf[index++] = 3*MAX_CTRL_PARAMS;
    for(int i=0; i < MAX_CTRL_PARAMS; i++)
    {
      u8TxBuf[index++] = i;
      u8TxBuf[index++]= (unsigned char)((params[i].value_curr_mem >>8) & 0xFF);
      u8TxBuf[index++]= (unsigned char)(params[i].value_curr_mem & 0xFF);
    }                
  }
  else
  {
    u8TxBuf[index++] = 3; // Data Payload length 
    u8TxBuf[index++] = param_index; // Param index
    u8TxBuf[index++]= (unsigned char)((params[param_index].value_curr_mem >>8) & 0xFF);
    u8TxBuf[index++]= (unsigned char)(params[param_index].value_curr_mem & 0xFF);        
  }
  UART2_SendDatawithCRC16(u8TxBuf, index); 
}
/*
 * This function forms the response packet with sensor data and send it on the UART2
 */
void Send_Sensors_Data(int sensor_index)
{
  int index =0;
  //Serial.println("Read Sensors Data");
  u8TxBuf[index++] = MOTOR_CONTROLLER_ADDR;  //Destinatin address
  u8TxBuf[index++] = READ_SENSORS_DATA_FC;  //Function code    
  if(sensor_index == READ_ALL_SENSORS_DATA)
  {
    u8TxBuf[index++] = 3*NUM_OF_SENSORS;  //Data Payload length
    //Serial.println("Read all sensors data");
    for(int i=0; i < NUM_OF_SENSORS; i++)
    {
      u8TxBuf[index++] = i;
      u8TxBuf[index++]= (unsigned char)((sensorOutputData[i].unitX10 >> 8) & 0x00FF);
      u8TxBuf[index++]= (unsigned char)(sensorOutputData[i].unitX10 & 0xFF);
    }
            
  }
  else
  {
    //Serial.println("Read Sensors" + String(CtrlRxBuf[DATA_PAYLOAD_INDEX]) + " data");        
    u8TxBuf[index++] = 3; //Data payuload length 
    u8TxBuf[index++] = sensor_index; //
    u8TxBuf[index++]= (unsigned char)((sensorOutputData[sensor_index].unitX10 >> 8) & 0x00FF);
    u8TxBuf[index++]= (unsigned char)(sensorOutputData[sensor_index].unitX10 & 0xFF);        
  }
  UART2_SendDatawithCRC16(u8TxBuf, index); 
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
bool UART2_IsCtrlPacketRecevied(unsigned char **Buff, int *Len)
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
        *Buff = UartRxBuff;   
        *Len = UARTIndex; 
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
    
  }
  return(StatusF);
}
/*
 * Function to verify the integrity of packet received from Motor control.
 */
bool CRCVerify(unsigned char *LenCtrlEventBuff, int EvetDataLen)
{
  return(true);
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
