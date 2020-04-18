//#include "calib_calc_m_c.h"

#define STPR_STP 0
#define STPR_ON  1
#define OXY_SOLE_CYL_OFF 2
#define OXY_SOLE_CYL_ONN  3
#define OXY_SOLE_HOS_O2_OFF 4
#define OXY_SOLE_HOS_O2_ONN  5
#define INH_SOLE_OFF 6
#define INH_SOLE_ONN  7
#define EXH_SOLE_OFF 8
#define EXH_SOLE_ONN  9
#define PK_PR_REL_OFF 10
#define PK_PR_REL_ONN  11
#define SET_TID_VOL  12
#define SET_BPM  13
#define SET_PK_PR 14
#define SET_FIO2  15
#define SET_IE_RATIO 16
#define SL_EN_PARM_EDT  17
#define SL_COM_PARM_EDT 18
#define INIT_MASTER  19
#define INIT_STPR_MOD 20
#define INIT_VALV_BLK  21
#define INIT_BREATH_DET  22

#define START_DELIM '$'
#define END_DELIM '&'
#define VENT_MAST "VM"
#define TIDAL_VOLUME tidl_volu.parm_name
#define RR resp_rate.parm_name
#define IER inex_rati.parm_name 
#define PARAM1 "P1"
#define PARAM2 "P2"
#define PARAM5 "P5"
int binaryNum[4];




typedef enum
{
  CTRL_INIT=0,
  CTRL_COMPRESSION,
  CTRL_COMPRESSION_HOLD,
  CTRL_EXPANSION,
  CTRL_EXPANSION_HOLD,
  CTRL_INHALE_DETECTION,
  CTRL_UNKNOWN_STATE,
}ControlStatesDef_T;


static const String commands[] =
{ "$VMST0000&",
  "$VMST0001&",
  "$VMO20100&",
  "$VMO20101&",
  "$VMO20200&",
  "$VMO20201&",
  "$VMSV0100&",
  "$VMSV0101&",
  "$VMSV0200&",
  "$VMSV0201&",
  "$VMSV0300&",
  "$VMSV0301&",
  "$VMP1xxxx&",
  "$VMP2xxxx&",
  "$VMP3xxxx&",
  "$VMP4xxxx&",
  "$VMP5xxxx&",
  "$VMPP0000&",
  "$VMPP1111&",
  "$VMIN0000&",
  "$VMIN0001&",
  "$VMIN0002&",
  "$VMIN0003&"
};
void decToBinary(int n);

// function to convert decimal to binary
void decToBinary(int n)
{
  // array to store binary number
  // counter for binary array
  int i = 0;
  while (n > 0) {
    // storing remainder in binary array
    binaryNum[i] = n % 2;
    n = n / 2;
    i++;
  }
}

bool inRange(unsigned low, unsigned high, unsigned x);

// Returns true if x is in range [low..high], else false
bool inRange(unsigned low, unsigned high, unsigned x)
{
  return  ((x - low) <= (high - low));
}
int getSensorValue(String paramName,int index);
int getParamterValue(String paramName,int index);
String getAllSensorsReading(String paramName);
String getGraphSensorsReading();
String getAllUIParamsReading(String paramName);
String getRequiredSensorsReading(String paramName, int binaryValue);



String getSensorReading(String paramName, int value);
