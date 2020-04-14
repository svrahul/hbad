#define START_DELIM '$'
#define END_DELIM '&'
#define VENT_MAST "VM"
#define CMD_PACKET_SIZE 10

/*
 * PACKET STUCTURE
 * <SDL><TID><FC><VALUES><EDL>
 * SDL; STart delimiter
 * EDL: End delimiter
 * TID: Target Identifier; VM for Ventilator Master. VS For Ventilator Slave
 * FC: Function Code
 */

typedef enum
{
  SENSORS_DATA=0,
  PARAMS_DATA,
}UartPacketTypeDef;

/*Packet definition for sending sensors data to graphics display module */

#define TOTAL_NUMBER_OF_SENSORS NUM_OF_SENSORS
#define TOTAL_NUMBER_OF_PARAMS  MAX_CTRL_PARAMS


#define PACKET_LEN_INDEX  0
#define FUNCTION_CODE_INDEX 1
#define NUMBER_ELEMENTS_INDEX 2
#define DATA_PAYLOAD_INDEX 3
#define SENSORS_CRC_2B_INDEX  (DATA_PAYLOAD_INDEX + TOTAL_NUMBER_OF_SENSORS*2) 
#define PARAMS_CRC_2B_INDEX  (DATA_PAYLOAD_INDEX + TOTAL_NUMBER_OF_PARAMS*2)

#define TOTAL_SENSORS_PACKET_LEN  (SENSORS_CRC_2B_INDEX+2)
#define TOTAL_PARAMS_PACKET_LEN (PARAMS_CRC_2B_INDEX+2)

#define SENSORS_DATA_FC     0x01
#define PARAMETERS_DATA_FC  0x02

#if (TOTAL_SENSORS_PACKET_LEN > TOTAL_PARAMS_PACKET_LEN)
#define TOTAL_PACKET_LEN TOTAL_SENSORS_PACKET_LEN
#else
#define TOTAL_PACKET_LEN TOTAL_PARAMS_PACKET_LEN
#endif

/*Global TX Buffer for sending the data over UART3 to Graphics display  module */
unsigned char u8TxBuffer[TOTAL_PACKET_LEN];

void UART3_SendDAQDataGraphicDisplay(UartPacketTypeDef ePacketType);
unsigned short crc16(unsigned char *data_p, unsigned short length);
 
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
