


struct sensor_parameter_t {
  unsigned short index;
  String parm_name;
  int current_val;
  int old_val;
  String units;
  int channel_num;
};

const sensor_parameter_t ps1_sense =    {1, "PS1",
                                       -1,
                                       -1,
                                       "mV",
                                        0
                                      };
const sensor_parameter_t ps2_sense =    {2, "PS2",
                                       -1,
                                       -1,
                                       "mV",
                                        1
                                      };
const sensor_parameter_t fio2_sense =   {3, "FiO2",
                                       0,
                                       0,
                                       "%",
                                        0
                                      };
const sensor_parameter_t dps1_sense =   {4, "DPS1",
                                       0,
                                       0,
                                       "mV",
                                        0
                                      };
const sensor_parameter_t dps2_sense =   {5, "DPS2",
                                       0,
                                       0,
                                       "mV",
                                       0
                                      };
static sensor_parameter_t sensor_params[] = {ps1_sense, ps2_sense, fio2_sense,dps1_sense, dps2_sense};
