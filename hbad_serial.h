#define START_DELIM '$'
#define END_DELIM '&'
#define CMD_PACKET_SIZE 10

/*
 * PACKET STUCTURE
 * <SDL><TID><FC><VALUES><EDL>
 * SDL; STart delimiter
 * EDL: End delimiter
 * TID: Target Identifier; VM for Ventilator Master. VS For Ventilator Slave
 * FC: Function Code
 */
