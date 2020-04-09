//Author: Venkata Rahul S, Team HBAD, 28-3-2020

#define MAX_CTRL_PARAMS 6
#define DBNC_INTVL_SW 500 // millisecs before switch debounce
#define DBNC_INTVL_ROT 50 // millisecs before rotation debounce
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
static const int analog_pins[] = {A0, A1, A2, A3, A4, A5};

static const int mode_loop_delays[] = {100, 100, 100, 100};
static const int mode_timeouts[] = {0, 0, 500, 0};

static const char CFG_INDICATOR ((char) 162);

static const String mode_headers[] = {"PRESS SELECT TO EDIT", "TURN KNOB TO SELECT", "TURN POT TO SETECT", "PRESS SELECT TO SAVE"};
struct ctrl_parameter_t {
  unsigned short index;
  String parm_name;
  int min_val;
  int max_val;
  String units;
  int incr;
  unsigned short param_type;
  int value_curr_mem;
  int value_from_sns;
  int value_new_pot;
};

const ctrl_parameter_t tidl_volu = {1, "Tidal Volu",
                                    200,
                                    600,
                                    "ml",50,
                                    PARAM_TYPE_UCFG,
                                    0, 0, 0
                                   };
const ctrl_parameter_t resp_rate =    {2, "Resp. rate",
                                       10,
                                       25,
                                       "BPM",1,
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t peak_press =   {3, "Peak press",
                                       40,
                                       60,
                                       UNIT_CMH2O,1,
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t inex_rati =    {4, "IER",
                                       1,
                                       3,
                                       "ratio",1,
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t fio2_perc =    {6, "FiO2",
                                       20,
                                       100,
                                       "%",20,
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };

const ctrl_parameter_t peep_pres =    {5, "PEEP",
                                       5,
                                       20,
                                       UNIT_CMH2O,5,
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };

const ctrl_parameter_t ps1_sense =    {7, "PS1",
                                       -1,
                                       -1,
                                       UNIT_CMH2O,-1,
                                       PARAM_TYPE_SENS,
                                       0, 0, 0
                                      };
const ctrl_parameter_t ps2_sense =    {8, "PS2",
                                       -1,
                                       -1,
                                       UNIT_CMH2O,-1,
                                       PARAM_TYPE_SENS,
                                       0, 0, 0
                                      };
static ctrl_parameter_t params[] = {tidl_volu, resp_rate, peak_press, fio2_perc, inex_rati, peep_pres, ps1_sense, ps2_sense};
