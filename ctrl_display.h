//Author: Venkata Rahul S, Team HBAD, 28-3-2020

#define MAX_CTRL_PARAMS 4
#define DBNC_INTVL_SW 500 // millisecs before switch debounce
#define DBNC_INTVL_ROT 50 // millisecs before rotation debounce
#define MAX_IDLE_AFTER_SEL 10000000 //uSecs
#define POT_TURN_MAX_WAIT_MILLIS 2000
#define DISPLAY_MODE 0
#define EDIT_MODE_ON 1
#define PAR_SELECTED_MODE 2
#define PAR_SAVE_MODE 3
#define POT_HIGH 1000

/*Let us start writing from the 16th memory location. 
 * This means Parameter 1 will be stored in locs with addr 16, 17, 18, 19
 * and Parameter 1 will be stored in 20, 21, 22, 23 and so on
*/

#define PARAM_STOR_SIZE 4 //4 bytes for float data
#define PARAM_TYPE_UCFG 1
#define PARAM_TYPE_SENS 2

static const int analog_pins[] = {A0, A1, A2, A3, A4, A5};

static const int mode_loop_delays[] = {100, 100, 100, 100};
static const int mode_timeouts[] = {0, 0, 500, 0};

static const char CFG_INDICATOR ((char) 162);

static const String mode_headers[] = {"PRESS SELECT TO EDIT", "TURN KNOB TO SELECT", "TURN POT TO SETECT", "PRESS SELECT TO SAVE"};
String param_names[] = {"Tidal Volu", "Resp Rate", "Peak Press", "IER", "PEEP", "FiO2", "PS1", "PS2"};
String param_units[] = {"ml", "bpm", "cmH2o", "#", "cmH2o", "%", "mmH2o", "mmH2o"};
int param_range_min[] = {150, 6, 37, 0, 5, 21, -1, -1};
int param_range_max[] = {800, 35, 60, 3, 20, 100, -1, -1};
int param_incr[] = {50, 1, 1, 1, 20, -1};
struct ctrl_parameter_t {
  unsigned short index;
  String parm_name;
  int min_val;
  int max_val;
  String units;
  unsigned short param_type;
  int value_curr_mem;
  int value_from_sns;
  int value_new_pot;
};

const ctrl_parameter_t tidl_volu = {1, param_names[0],
                                    param_range_min[0],
                                    param_range_max[0],
                                    param_units[0],
                                    PARAM_TYPE_UCFG,
                                    0, 0, 0
                                   };
const ctrl_parameter_t resp_rate =    {2, param_names[1],
                                       param_range_min[1],
                                       param_range_max[1],
                                       param_units[1],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t peak_press =   {3, param_names[2],
                                       param_range_min[2],
                                       param_range_max[2],
                                       param_units[2],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t inex_rati =    {4, param_names[3],
                                       param_range_min[3],
                                       param_range_max[3],
                                       param_units[3],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t peep_pres =    {5, param_names[4],
                                       param_range_min[4],
                                       param_range_max[4],
                                       param_units[4],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };

const ctrl_parameter_t fio2_perc =    {6, param_names[5],
                                       param_range_min[5],
                                       param_range_max[5],
                                       param_units[5],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };

const ctrl_parameter_t ps1_sense =    {7, param_names[6],
                                       param_range_min[6],
                                       param_range_max[6],
                                       param_units[6],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
const ctrl_parameter_t ps2_sense =    {8, param_names[7],
                                       param_range_min[7],
                                       param_range_max[7],
                                       param_units[7],
                                       PARAM_TYPE_UCFG,
                                       0, 0, 0
                                      };
static ctrl_parameter_t params[] = {tidl_volu, resp_rate, peak_press, inex_rati, peep_pres, fio2_perc, ps1_sense, ps2_sense};
