
int8_t ps1_current, ps1_old, ps2_current, ps2_old,fio2_current,fio2_old,dps1_current,dps1_old, dps2_current, dps2_old;

sensor_parameter_t get_PS1(sensor_parameter_t _params);
sensor_parameter_t get_PS2(sensor_parameter_t s_params);
sensor_parameter_t get_FiO2(sensor_parameter_t s_params);
sensor_parameter_t get_DPS1(sensor_parameter_t s_params);
sensor_parameter_t get_DPS2(sensor_parameter_t s_params);
  

// To get PS1 Value in sensor_parameter_t
sensor_parameter_t get_PS1(sensor_parameter_t s_params){
ps1_old = ps1_current;
ps1_current= PS_ReadSensorMilliVolt(PS1);
s_params.current_val = ps1_current;
s_params.old_val = ps1_old;
return s_params;
 
}

// To get PS2 Value in sensor_parameter_t
sensor_parameter_t get_PS2(sensor_parameter_t s_params){ 
ps2_old = ps2_current;
ps2_current = PS_ReadSensorMilliVolt(PS2);
s_params.current_val = ps2_current;
s_params.old_val = ps2_old;
return s_params;

}

// To get FiO2 Value in sensor_parameter_t
sensor_parameter_t get_FiO2(sensor_parameter_t s_params){ 
fio2_old = fio2_current;
fio2_current = PS_ReadSensorMilliVolt(O2);
s_params.current_val = fio2_current;
s_params.old_val = fio2_old;
return s_params;

}

// To get diff pressure sensor 1 Value in sensor_parameter_t
sensor_parameter_t get_DPS1(sensor_parameter_t s_params){ 
dps1_old = dps1_current;
dps1_current = PS_ReadSensorMilliVolt(DPS1);
s_params.current_val = dps1_current;
s_params.old_val = dps1_old;
return s_params;

}

// To get diff pressure sensor 2 Value in sensor_parameter_t
sensor_parameter_t get_DPS2(sensor_parameter_t s_params){ 
dps2_old = dps2_current;
dps2_current = PS_ReadSensorMilliVolt(DPS2);
s_params.current_val = dps2_current;
s_params.old_val = dps2_old;
return s_params;

}
