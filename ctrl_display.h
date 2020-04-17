//Author: Venkata Rahul S, Team HBAD, 28-3-2020

#define MAX_CTRL_PARAMS 6

//#define MAX_CTRL_PARAMS sizeof(params)/ sizeof(params[0])
#define DBNC_INTVL_SW 500 // millisecs before switch debounce
#define DBNC_INTVL_ROT 100 // millisecs before rotation debounce
#define MAX_IDLE_AFTER_SEL 10000000 //uSecs
#define POT_TURN_MAX_WAIT_MILLIS 2000
#define DISPLAY_MODE 0
#define EDIT_MODE 1
#define PAR_SELECTED_MODE 2
#define PAR_SAVE_MODE 3
#define POT_HIGH 1000

/*Let us start writing from the 16th memory location.
   This means Parameter 1 will be stored in locs with addr 16, 17, 18, 19
   and Parameter 1 will be stored in 20, 21, 22, 23 and so on
*/

#define PARAM_STOR_SIZE 4 //4 bytes for float data
#define PARAM_TYPE_UCFG 1
#define PARAM_TYPE_SENS 2
#define UNIT_CMH2O "cmH20"

static const int mode_loop_delays[] = {100, 100, 100, 100};
static const int mode_timeouts[] = {0, 0, 500, 0};
#define EMPTY_TWENTY_STR "                    "
#define EMPTY_FIVE_STR "     "
static const char CFG_INDICATOR ((char) 162);

static const String mode_headers[] = {"PRESS SELECT TO EDIT", EMPTY_TWENTY_STR, EMPTY_TWENTY_STR, "PRESS SELECT TO SAVE"};
struct ctrl_parameter_t {
  unsigned short index;
  String parm_name;
  int readPortNum;
  int min_val;
  int max_val;
  String units;
  int incr;
  int value_curr_mem;
  int value_new_pot;
};

typedef enum
{
  TIDAL_VOL=0,
  BPM,
  PEAK_PRES,
  FIO2_PERC,
  IE_RATIO,
  PEEP_PRES
};
const ctrl_parameter_t tidl_volu = {0, "TV", TIDAL_VOLUME_PIN,
                                    200, 600,
                                    "ml   ", 50,
                                    0, 0
                                   };
const ctrl_parameter_t resp_rate =    {1, "RR", RR_PIN,
                                       5, 35,
                                       "BPM  ", 1,
                                       0, 0
                                      };
const ctrl_parameter_t peak_press =   {2, "PAW", PMAX_PIN,
                                       40, 60,
                                       UNIT_CMH2O, 1,
                                       0, 0
                                      };
const ctrl_parameter_t fio2_perc =    {3, "FiO2", FiO2_PIN,
                                       20, 100,
                                       "%    ", 20,
                                       0, 0
                                      };
const ctrl_parameter_t inex_rati =    {4, "IE", DISP_ENC_CLK, //READ THROUGH ENCODER
                                       1, 3,
                                       "ratio", 1,
                                       0, 0
                                      };
const ctrl_parameter_t peep_pres =    {5, "PP", DISP_ENC_CLK, //READ THROUGH ENCODER
                                       5, 20,
                                       UNIT_CMH2O, 5,
                                       0, 0
                                      };

static ctrl_parameter_t params[] = {tidl_volu, resp_rate, peak_press, fio2_perc, inex_rati, peep_pres};

#define SAVE_FLAG " SAVE "
#define SAVE_FLAG_CHOSEN "<SAVE>"
#define CANC_FLAG " CANCEL "
#define CANC_FLAG_CHOSEN "<CANCEL>"
