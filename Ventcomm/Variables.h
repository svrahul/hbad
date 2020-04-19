#ifndef _Variable_H    // Put these two lines at the top of your file.
#define _Variable_H    // (Use a suitable name, usually based on the file name.)


#define MIN_RPM_NO_ACCEL 300.0

//this machanisum has 1:2 ratio adjusted formulas to reflect below data after belt ratio
volatile int Par_editstat;


volatile float tidal_volume = 600.0;
volatile float BPM = 12.0;
volatile int peak_prsur = 12.0;
volatile float FiO2 = 21.0;
volatile float IER = 102;
volatile float inhale_ratio = 1.0;
volatile float exhale_ratio = 2.0;
volatile float Stroke_length = 90.0;
volatile float PEEP = 20.0;

volatile int tidal_volume_new = 600.0;
volatile float BPM_new = 12.0;
volatile float IER_new = 2.0;  //I=1 and E=2
volatile float inhale_ratio_new = 1.0;
volatile float exhale_ratio_new = 2.0;
volatile float Stroke_length_new = 90.0;
volatile float PEEP_new = 20.0;


volatile float compression_min_speed = MIN_RPM_NO_ACCEL;
volatile float expansion_min_speed = MIN_RPM_NO_ACCEL;

volatile float micro_stepping = 800.0;
volatile float home_speed_value = 200.0;        //menu

volatile float compression_speed = 300.0;
volatile float expansion_speed = 200.0;


int comcnt = 0;
String rxdata = "";


//this 1.8 degree step motor so 200 steps for 360 degree
volatile float run_pulse_count_1_full_movement = 0;
volatile float run_pulse_count_1_piece_compression = 0;
volatile float run_pulse_count_1_piece_expansion = 0;

volatile long OCR1A_var;
volatile long TCCR1B_var;
volatile float run_pulse_count = 0.0;
volatile float run_pulse_count_temp = 0.0;
volatile long motion_profile_count_temp = 0;
volatile float compression_step_array[21];
volatile float expansion_step_array[21];
volatile float compression_speed_array[21];
volatile float expansion_speed_array[21];
volatile long TCCR1B_comp_array[21];
volatile long OCR1A_comp_array[21];
volatile long TCCR1B_exp_array[21];
volatile long OCR1A_exp_array[21];

long c_start_millis = 0;
long c_end_millis = 0;
long e_start_millis = 0;
long e_end_millis = 0;

volatile boolean Emergency_motor_stop = false;
volatile boolean run_motor = false;
volatile boolean cycle_start = false;
volatile boolean home_cycle = false;

volatile boolean comp_start = false;
volatile boolean comp_end = false;
volatile boolean exp_start = false;
volatile boolean exp_end = false;

#define MOTOR_RUN_PIN  10
#define MOTOR_STEP_PIN  9
#define MOTOR_DIR_PIN  8
#define HOME_SENSOR_PIN  7
#define INHALE_RELEASE_VLV_PIN 6  //Normally closed Valve
#define INHALE_VLV_PIN 5   //Normally open Valve
#define EXHALE_VLV_PIN 4   //Normally open Valve
#define O2Cyl_VLV_PIN 3   //Normally closed Valve
#define O2Hln_VLV_PIN 2  //Normally closed Valve




#define COMP_DIR HIGH //ACW
#define EXP_DIR LOW   //CW

#define LEAD_SCREW_PITCH 8.0

#define CURVE_COMP_STEPS 21
#define CURVE_EXP_STEPS 21



//int Inh_vlv = 53;
//int Exh_vlv = 52;
//int O2Cyl_vlv = 51;
//int O2Hln_vlv = 50;

//Normally Opened
#define EXHALE_VLV_OPEN()  digitalWrite(EXHALE_VLV_PIN, LOW)
#define EXHALE_VLV_CLOSE() digitalWrite(EXHALE_VLV_PIN, HIGH)
#define INHALE_VLV_OPEN()  digitalWrite(INHALE_VLV_PIN, LOW)
#define INHALE_VLV_CLOSE() digitalWrite(INHALE_VLV_PIN, HIGH)

//Normally closed
#define INHALE_RELEASE_VLV_OPEN()  digitalWrite(INHALE_RELEASE_VLV_PIN, HIGH)
#define INHALE_RELEASE_VLV_CLOSE() digitalWrite(INHALE_RELEASE_VLV_PIN, LOW)
#define O2Cyl_VLV_OPEN()  digitalWrite(O2Cyl_VLV_PIN, HIGH)
#define O2Cyl_VLV_CLOSE() digitalWrite(O2Cyl_VLV_PIN, LOW)
#define O2Hln_VLV_OPEN()  digitalWrite(O2Hln_VLV_PIN, HIGH)
#define O2Hln_VLV_CLOSE() digitalWrite(O2Hln_VLV_PIN, LOW)




//// Place your main header code here.
//float P1;
//float P2;
//int PrsurIn1 = A3;
//int PrsurIn2 = A4;
//int chno;
//int samples;

//const int RunningAverageCount = 30;
//float RunningAverageBuffer[RunningAverageCount];
//int NextRunningAverage;
//float RunningAverageValue;

// select the input pin for the potentiometer
//int BPM_POT_PIN = A0;
//int Tidal_vol_POT_PIN = A1;
//int Tidal_ANALOG_PIN = A2;

#define EXPANSION_HOLD_TIME 100
#define COMPRESSION_HOLD_TIME 500
typedef enum
{
  CTRL_IDLE=0,
  CTRL_COMPRESSION,
  CTRL_COMPRESSION_HOLD,
  CTRL_EXPANSION,
  CTRL_EXPANSION_HOLD,
  CTRL_INHALE_DETECTION,
  CTRL_UNKNOWN_STATE,
}ControlStatesDef_T;

unsigned long int StateStartMillis=0;

// Control statemachine gloabl variable
ControlStatesDef_T geCtrlState = CTRL_IDLE;
bool bSendInitCommand = false;
bool bCommandSendFlag = false;

typedef enum{
  PS1,
  PS2,
  DPS1,
  DPS2,
  O2,
  NUM_OF_SENSORS,
}sensor_e;
/*
 * Control parameters index
 */
typedef enum
{
  TIDAL_VOL=0,
  BREATHS_PM,
  PEAK_PRES,
  FIO2_PERC,
  IE_RATIO,
  PEEP_PRES,
  MAX_CTRL_PARAMS
}ControlParams_T;

typedef struct{
  unsigned int mV;
  unsigned int unitX10;
}sensorOutputDataT;

/*
 * main output sensor data global array populated in timer context.
 */
sensorOutputDataT sensorOutputData[NUM_OF_SENSORS];

int CtrlParams[MAX_CTRL_PARAMS];

#endif // _Variable_H    // Put this line at the end of your file.
