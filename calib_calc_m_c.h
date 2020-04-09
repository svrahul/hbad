
//#include <EEPROM.h>
#include "hbad_memory.h"
#include "ventilator_sensors.h"


//extern extEEPROM hbad_mem(kbits_256, 1, 32, EEPROM_I2C_ADDR);

//global data
//slope(m) and constant(c) for all the sensors.
float m_o2, c_o2, m_pg1, c_pg1, m_pg2, c_pg2, m_dpg1, c_dpg1, m_dpg2, c_dpg2;
boolean addr_default_data_write_to_eeprom_on_first_program_load = 10;
int * addr_o2_y_data = 12;
int * addr_pg1_y_data = 50;

unsigned int numOfSamples_o2 = 5;
unsigned int numOfSamples_pg1 = 19;


//control flags
boolean debug_prints = false;
boolean execute_cal = true;
boolean execute_get_val = true;


/*
 * function declarations
 */
void write_to_eeprom(unsigned int numOfIntWrites, int * addr, int *val);
//void calib_calculate_m_c(int numOfSamples, float *x_val, char * addr, float *m, float *c);
//float getY(float);
//float getX(float);


void setup_calib_calc_m_c() {
  // put your setup code here, to run once:
  ctrl_parameter_t param;
  //Serial.begin(9600);
  //Serial.println("starting!");
  param.index = addr_default_data_write_to_eeprom_on_first_program_load;
  retrieveParam(param);
  boolean default_data_write_to_eeprom_on_first_program_load = param.value_new_pot;
  if (default_data_write_to_eeprom_on_first_program_load)
  {
    float y_o2[] = {0.377, 1.088, 1.75, 2.11, 4.812};
    float y_pg1[] = {0.2, 0.22, 0.24, 0.26, 0.28, 0.3, 0.32, 0.33, 0.35, 0.37, 0.39, 0.41, 0.43, 0.45, 0.47, 0.49, 0.5, 0.52, 0.54};
 
    unsigned int int_y_o2[] = {377, 1088, 1750, 2110, 4812};
    unsigned int int_y_pg2[] = {200, 220, 240, 260, 280, 300, 320, 330, 350, 370, 390, 410, 430, 450, 470, 490, 500, 520, 540};
    
    write_to_eeprom(numOfSamples_o2*2, addr_o2_y_data, int_y_o2);
    write_to_eeprom(numOfSamples_pg1*2, addr_pg1_y_data, int_y_pg2);
    param.value_new_pot = false;
    storeParam(param);
  }
}

/*
 * main function to calibrate and get m (slope) and c (constant) after curve fitting
 * read y data from eeprom
 * convert from byte to float
 * use in algo to calc m and c values.
 */
void calib_calculate_m_c(unsigned int numOfSamples, float *x_val, int * addr, float *m, float *c)
{
  float sigmaX=0, sigmaY=0, sigmaXX=0, sigmaXY=0, denominator, y_val;
  ctrl_parameter_t param;
  unsigned int index;
  unsigned char value1, value2;
  Serial.println("printing values from addr");
  for (index = 0; index < numOfSamples; index++)
  {
    param.index = addr;
    retrieveParam(param);
    y_val = ((float)param.value_new_pot)/1000;
    //value1 = EEPROM.read(addr);
    //addr+=1;
    //value2 = EEPROM.read(addr);
    //y_val = ((float)((value2<<8)+(value1)))/1000;
    addr+=1;
    
    if (debug_prints)
    {
      Serial.print("addr ");Serial.print((int)addr, HEX);Serial.print(" = ");
      //Serial.print(value2, DEC); Serial.print(" "); Serial.print(value1, DEC);
      Serial.print(" = in float "); Serial.println(y_val,5);
    }
    sigmaX += x_val[index];
    sigmaY += y_val;
    sigmaXX += x_val[index]*x_val[index];
    sigmaXY += x_val[index]*y_val;
  }
  denominator = (numOfSamples*sigmaXX) - (sigmaX*sigmaX);
  if (denominator != 0)
  {
    *m = ((numOfSamples*sigmaXY) - (sigmaX*sigmaY)) / denominator;
    *c = ((sigmaY*sigmaXX) - (sigmaX*sigmaXY)) / denominator;
  }
  else
  {
    Serial.println("Error denominator 0, calibration failed!");
    *m=0;
    *c=0;
  }
}

/*
 * this function needs to be called upon boot and 
 * whenever online calibration is done
 * 
 * m and c values for all sensors are calculated and 
 * stored as global values.
 */
void calc_m_c_for_all_sensors()
{
    float const x_o2[numOfSamples_o2] = {0, 21.6, 28, 40, 100};
    float const x_pg1[numOfSamples_pg1] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95};

    calib_calculate_m_c(numOfSamples_o2, x_o2, addr_o2_y_data, &m_o2, &c_o2);
    calib_calculate_m_c(numOfSamples_pg1, x_pg1, addr_pg1_y_data, &m_pg1, &c_pg1);
    //calib_calculate_m_c(numOfSamples_pg2, x_pg1, y_pg2, &m_pg2, &c_pg2);
    //calib_calculate_m_c(numOfSamples_dpg1, x_dpg, y_dpg1, &m_dpg1, &c_dpg1);
    //calib_calculate_m_c(numOfSamples_dpg2, x_dpg, y_dpg2, &m_dpg2, &c_dpg2);
}

/*
 * getX will return value of x corresponding to value y,m,c
 * x = (y-c)/m
 */
float getX(float y, float m, float c)
{
  float x = 0;
  if (m != 0)
  {
    x = (y-c)/m;
  }
  return x;
}

/*
 * getY will return value of x corresponding to value y,m,c
 * y = mx+c
 */
float getY(float x, float m, float c)
{
  float y;
  y = (m*x + c);
  return y;
}

float get_o2_voltage(float y_percentage)
{
  float x;
  x = getX(y_percentage,m_o2,c_o2);
  return x;
}

float get_pressure_voltage(float y_unit, int channel)
{
  float x;
  switch(channel)
  {
    case PS1:
      x = getX(y_unit,m_pg1,c_pg1);
      break;
    case PS2:
      x = getX(y_unit,m_pg2,c_pg2);
      break;
    case DPS1:
      x = getX(y_unit,m_dpg1,c_dpg1);
      break;
    case DPS2:
      x = getX(y_unit,m_dpg2,c_dpg2);
      break;
    default:
      break;      
  }
  return x;
}

/*
 * get o2 percentage in float for given voltage reading of o2 sensor
 */
float get_o2_percentage(float x_voltage)
{
  float y;
  y = getY(x_voltage,m_o2,c_o2);
  return y;
}
/*
 * get pg1 units in float for given voltage reading of pg1 sensor
 */
float get_pressure_value(float x_voltage, int channel)
{
  float y;
  switch(channel)
  {
    case PS1:
      y = getY(x_voltage,m_pg1,c_pg1);
      break;
    case PS2:
      y = getY(x_voltage,m_pg2,c_pg2);
      break;
    case DPS1:
      y = getY(x_voltage,m_dpg1,c_dpg1);
      break;
    case DPS2:
      y = getY(x_voltage,m_dpg2,c_dpg2);
      break;
    default:
      break;      
  }
  return y;
}


void write_to_eeprom(unsigned int numOfIntWrites, int * addr, int *val)
{
  unsigned int index;
  ctrl_parameter_t param;
  //Serial.println("writing data");
  for (index=0; index<numOfIntWrites; index++)
  {
    param.value_new_pot = *val;
    param.index = addr;
    storeParam(param);
    //EEPROM.write(addr, *val);
    if (debug_prints)
    {
      Serial.print("addr "); Serial.print((int)addr, HEX);Serial.print(" = ");Serial.print(*val, DEC);Serial.println(".");
    }
    addr++;
    val++;
  }
}


void test_loop_calib_calc_m_c() {
  if (execute_cal)
  {
    float const x_o2[numOfSamples_o2] = {0, 21.6, 28, 40, 100};
    float const x_pg1[numOfSamples_pg1] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95};
    execute_cal=false;
    // calibrate o2 data
    //get o2 y data from eeprom
    calib_calculate_m_c(numOfSamples_o2, x_o2, addr_o2_y_data, &m_o2, &c_o2);
    //calibrate pg1 data
  
  
    calib_calculate_m_c(numOfSamples_pg1, x_pg1, addr_pg1_y_data, &m_pg1, &c_pg1);
    
    //calibrate pg2 data
    //calib_calculate_m_c(numOfSamples_pg2, x_pg1, y_pg2, &m_pg2, &c_pg2);
    
    //calibrate dpg1 data
    //calib_calculate_m_c(numOfSamples_dpg1, x_dpg, y_dpg1, &m_dpg1, &c_dpg1);
    
    //calibrate dpg1 data
    //calib_calculate_m_c(numOfSamples_dpg2, x_dpg, y_dpg2, &m_dpg2, &c_dpg2);
    //if (debug_prints)
    {
      Serial.print("computed m O2:");
      Serial.print(m_o2,5);
      Serial.print(",  computed c O2:");
      Serial.print(c_o2,5);
      Serial.println(".");
      Serial.print("computed m PG1:");
      Serial.print(m_pg1,5);
      Serial.print(",  computed c PG1:");
      Serial.print(c_pg1,5);
      Serial.println(".");
    }
  }
  if (execute_get_val)
  {
    float x,y;
    execute_get_val=false;
    Serial.println("for X:     \t Y");
    for (x=0.0; x<100; x+=10)
    {
      y = getY(x,m_o2,c_o2);
      Serial.print(x,5);
      Serial.print(":\t");
      Serial.println(y,5);
    }
  
    Serial.println("for Y:     \t X");
    for (y=0.0; y<5; y+=0.5)
    {
      x = getX(y,m_o2,c_o2);
      Serial.print(y,5);
      Serial.print(":\t");
      Serial.println(x,5);
    }
  }
}
