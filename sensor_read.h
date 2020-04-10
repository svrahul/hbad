
boolean get_PS1(sensor_parameter_t *s_params);
boolean get_PS2(sensor_parameter_t *s_params);
boolean get_FiO2(sensor_parameter_t *s_params);
boolean get_DPS1(sensor_parameter_t *s_params);
boolean get_DPS2(sensor_parameter_t *s_params);
  

// To get PS1 Value in sensor_parameter_t
boolean get_PS1(sensor_parameter_t *s_params){
s_params->old_val = s_params->current_val;
s_params->current_val= PS_ReadSensorMilliVolt(PS1);
return true;
 
}

// To get PS2 Value in sensor_parameter_t
 boolean get_PS2(sensor_parameter_t *s_params){ 
s_params->old_val = s_params->current_val;
s_params->current_val= PS_ReadSensorMilliVolt(PS2);
return true;

}

// To get FiO2 Value in sensor_parameter_t
boolean get_FiO2(sensor_parameter_t *s_params){ 
s_params->old_val = s_params->current_val;
s_params->current_val= PS_ReadSensorMilliVolt(O2);
return true;

}

// To get diff pressure sensor 1 Value in sensor_parameter_t
boolean get_DPS1(sensor_parameter_t *s_params){ 
s_params->old_val = s_params->current_val;
s_params->current_val= PS_ReadSensorMilliVolt(DPS1);
return true;

}

// To get diff pressure sensor 2 Value in sensor_parameter_t
boolean get_DPS2(sensor_parameter_t *s_params){ 
s_params->old_val = s_params->current_val;
s_params->current_val= PS_ReadSensorMilliVolt(DPS2);
return true;

}
