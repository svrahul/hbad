


#include "hbad_memory.h"
#include "ventilator_sensors.h"

#define AVOID_EEPROM 0
#define DEBUG_PRINTS 0




/*
 * global data
 */

typedef struct {
  int numOfSamples;
  int * xVoltage;
  int * yUnit;
  int * eepromAddr;
  float m;
  float c;
}sensorDataT;

typedef struct{
  unsigned int mV;
  unsigned int unitX10;
}sensorOutputDataT;

/*
 * main output sensor data global array populated in timer context.
 */
sensorOutputDataT sensorOutputData[NUM_OF_SENSORS];

int gSensorDataUpdated =0;

//slope(m) and constant(c) for all the sensors.
float m_o2, c_o2, m_pg1, c_pg1, m_pg2, c_pg2, m_dpg1, c_dpg1, m_dpg2, c_dpg2;

int const xO2UnitX10[NUM_OF_SAMPLES_O2] = {0, 216, 280, 400, 1000};
//float y_o2[] = {0.377, 1.088, 1.75, 2.11, 4.812};
unsigned int yO2VoltX1000[] = {377, 1088, 1750, 2110, 4812};

int const xPs1Unitx10[NUM_OF_SAMPLES_PS1] = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950};
//float y_pg1[] = {0.2, 0.22, 0.24, 0.26, 0.28, 0.3, 0.32, 0.33, 0.35, 0.37, 0.39, 0.41, 0.43, 0.45, 0.47, 0.49, 0.5, 0.52, 0.54};
unsigned int yPs1VoltX1000[] = {200, 220, 240, 260, 280, 300, 320, 330, 350, 370, 390, 410, 430, 450, 470, 490, 500, 520, 540};


#if AVOID_EEPROM
boolean addr_default_data_write_to_eeprom_on_first_program_load[1];
int addr_o2_y_data[NUM_OF_SAMPLES_O2];
int addr_pg1_y_data[NUM_OF_SAMPLES_PS1];
#else
boolean * addr_default_data_write_to_eeprom_on_first_program_load = EEPROM_DEFAULT_DATA_WRITE_ADDR;
int * addr_o2_y_data = EEPROM_O2_CALIB_ADDR;
int * addr_pg1_y_data = EEPROM_PG1_CALIB_ADDR;
#endif








sensorDataT sensorData[NUM_OF_SENSORS] = 
{
  [PS1] = {    NUM_OF_SAMPLES_PS1, xPs1Unitx10, yPs1VoltX1000,addr_pg1_y_data, 0.0, 0.0  },
  [PS2] = {    NUM_OF_SAMPLES_PS1, xPs1Unitx10, yPs1VoltX1000,addr_pg1_y_data, 0.0, 0.0  },//PS2=PS1
  [DPS1] = {    NUM_OF_SAMPLES_DPS1, NULL, NULL, NULL, 0.0, 0.0  },
  [DPS2] = {    NUM_OF_SAMPLES_DPS2, NULL, NULL, NULL, 0.0, 0.0  },
  [O2]={    NUM_OF_SAMPLES_O2, xO2UnitX10, yO2VoltX1000, addr_o2_y_data,0.0, 0.0  },
};

/*
 * function declarations
 */
void write_to_eeprom(unsigned int numOfIntWrites, int * addr, int *val);
void calib_calculate_m_c(unsigned int numOfSamples, float *x_val, int * addr, float *m, float *c);
void calc_m_c_for_all_sensors();


void setup_calib_calc_m_c() {
  // put your setup code here, to run once:
  boolean default_data_write_to_eeprom_on_first_program_load;
  int index;
  Serial.println("starting calib_calc_m_c setup!");
  #if AVOID_EEPROM
  default_data_write_to_eeprom_on_first_program_load = true;
  #else
  default_data_write_to_eeprom_on_first_program_load = retrieveCalibParam(
    addr_default_data_write_to_eeprom_on_first_program_load);
  #endif
  
  //if (default_data_write_to_eeprom_on_first_program_load)
  {
    for (index=0; index<NUM_OF_SENSORS; index++)
    {
      write_to_eeprom(sensorData[index].numOfSamples,
          sensorData[index].eepromAddr,
          sensorData[index].yUnit);
    }
    
    #if AVOID_EEPROM
    *(addr_default_data_write_to_eeprom_on_first_program_load)=false;
    #else
    storeCalibParam(addr_default_data_write_to_eeprom_on_first_program_load, false);
    Serial.println("Default data written to EEPROM");
    #endif
  }
  Serial.println("Done calib_calc_m_c setup!");
  calc_m_c_for_all_sensors();
  ADS1115_init();
}

/*
 * main function to calibrate and get m (slope) and c (constant) after curve fitting
 * read y data from eeprom
 * convert from byte to float
 * use in algo to calc m and c values.
 */
void calib_calculate_m_c(unsigned int numOfSamples, int *x_val, int * addr, float *m, float *c)
{
  float x,sigmaX=0, sigmaY=0, sigmaXX=0, sigmaXY=0, denominator, y_val;
  unsigned int index;
  int value;
  for (index = 0; index < numOfSamples; index++)
  {
#if AVOID_EEPROM
    y_val = ((float)(*addr))/1000;
#else
    value = retrieveCalibParam(addr);
    y_val = ((float)value)/1000;
#endif
    addr+=1;
    
    #if DEBUG_PRINTS
      Serial.print("addr ");Serial.print((int)addr, HEX);Serial.print(" = ");
      //Serial.print(value2, DEC); Serial.print(" "); Serial.print(value1, DEC);
      Serial.print(value, DEC);
      Serial.print(" = in float "); Serial.println(y_val,5);
    #endif
    x = (float(x_val[index]))/10;
    sigmaX += x;
    sigmaY += y_val;
    sigmaXX += x * x;
    sigmaXY += x * y_val;
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
    int index;
    float m,c;
    Serial.println("starting calc_m_c_for_all_sensors!");
    for (index = 0; index< NUM_OF_SENSORS; index++)
    {
      if ((sensorData[index].numOfSamples == 0)||
          (sensorData[index].xVoltage == NULL)||
          (sensorData[index].yUnit == NULL))
      {
        continue;
      }
      calib_calculate_m_c(
        sensorData[index].numOfSamples,
        sensorData[index].xVoltage,
        sensorData[index].eepromAddr,
        &(sensorData[index].m), 
        &(sensorData[index].c));
      Serial.print("sensor:");Serial.print(index);
      Serial.print(", computed m:"); Serial.print(sensorData[index].m,5);
      Serial.print(", c:");Serial.println(sensorData[index].c,5);
      
    }
    Serial.println("Done calc_m_c_for_all_sensors!");
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

int getSensorMilliVoltage(sensor_e sensor, int xSensorUnitValueX10)
{
  int y = 0;
  float y_float;
  if (sensor < NUM_OF_SENSORS)
  {
    y_float = getY(((float)xSensorUnitValueX10)/10,
            sensorData[sensor].m,
            sensorData[sensor].c);
    y = (int)(y_float * 1000);
  }
  #if DEBUG_PRINTS
  Serial.print("sensor:");Serial.print(sensor);Serial.print(": ");Serial.print(xSensorUnitValueX10,3);
  Serial.print("V = ");Serial.print(y,3);Serial.println(" (x1000).");
  Serial.print(sensorData[sensor].m,5);Serial.print("m. c:");Serial.println(sensorData[sensor].c,5);
  #endif
  return y;
}

int getSensorUnitsx10(sensor_e sensor, int yMilliVoltageValue)
{
  int x = 0;
  float x_float, y;
  
  if (sensor < NUM_OF_SENSORS)
  {
    y = ((float)yMilliVoltageValue)/1000;
    x_float = getX(y,
            sensorData[sensor].m,
            sensorData[sensor].c);
    x = (int)(x_float*10);
  }
  #if DEBUG_PRINTS
  Serial.print("sensor:");Serial.print(sensor);Serial.print(", ");
  Serial.print(yMilliVoltageValue);Serial.print("mV = ");Serial.print(x);Serial.println(" (x10).");
  Serial.print(sensorData[sensor].m,5);Serial.print("m. c:");Serial.println(sensorData[sensor].c,5);
  #endif
  return x;
}



void write_to_eeprom(unsigned int numOfIntWrites, int * addr, int *val)
{
  unsigned int index;
  if ((numOfIntWrites == 0)||
      (addr == NULL))
  {
    return;
  }
  //Serial.println("writing data");
  for (index=0; index<numOfIntWrites; index++)
  {
    #if AVOID_EEPROM
    *addr = *val;
    #else
    storeCalibParam(addr,*val);
    #endif
    #if DEBUG_PRINTS
      Serial.print("addr "); Serial.print((int)addr, HEX);Serial.print(" = ");Serial.print(*val, DEC);Serial.println(".");
    #endif
    addr++;
    val++;
  }
}

//control flags
boolean execute_cal = true;
boolean execute_get_val = true;

void test_loop_calib_calc_m_c() {
  if (execute_cal)
  {
    float const x_o2[NUM_OF_SAMPLES_O2] = {0, 21.6, 28, 40, 100};
    float const x_pg1[NUM_OF_SAMPLES_PS1] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95};
    execute_cal=false;
    // calibrate o2 data
    //get o2 y data from eeprom
    //calib_calculate_m_c(NUM_OF_SAMPLES_O2, x_o2, addr_o2_y_data, &m_o2, &c_o2);
    //calibrate pg1 data
  
  
    //calib_calculate_m_c(NUM_OF_SAMPLES_PS1, x_pg1, addr_pg1_y_data, &m_pg1, &c_pg1);
    
    //calibrate pg2 data
    //calib_calculate_m_c(numOfSamples_pg2, x_pg1, y_pg2, &m_pg2, &c_pg2);
    
    //calibrate dpg1 data
    //calib_calculate_m_c(numOfSamples_dpg1, x_dpg, y_dpg1, &m_dpg1, &c_dpg1);
    
    //calibrate dpg1 data
    //calib_calculate_m_c(numOfSamples_dpg2, x_dpg, y_dpg2, &m_dpg2, &c_dpg2);
    #if DEBUG_PRINTS
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
    #endif
  }
  if (execute_get_val)
  {
    float x,y;
    execute_get_val=false;
    Serial.println("for X:     \t Y");
    for (x=0.0; x<100; x+=10)
    {
      y = getY(x,m_pg1,c_pg1);
      Serial.print(x,5);
      Serial.print(":\t");
      Serial.println(y,5);
    }
  
    Serial.println("for Y:     \t X");
    for (y=0.0; y<0.5; y+=0.05)
    {
      x = getX(y,m_pg1,c_pg1);
      Serial.print(y,5);
      Serial.print(":\t");
      Serial.println(x,5);
    }
  }
}

int PS_ReadSensor(int Channel)
{
  float SensorVolt;
  if (Channel == O2)
  {
    SensorVolt=ADC_ReadVolageOnATMega2560(OXYGEN_ANALOG_PIN);
  }
  else
  {
    SensorVolt = ADS1115_ReadVolageOverI2C(Channel);
  }
  return(int(SensorVolt*1000));
}

int PS_ReadSensorMilliVolt(int Channel)
{
  if (Channel<NUM_OF_SENSORS)
  {
    return(sensorOutputData[Channel].mV);
  }
  return 0;
}

int PS_ReadSensorValueX10(int Channel)
{
  if (Channel<NUM_OF_SENSORS)
  {
    return(sensorOutputData[Channel].unitX10);
  }
  return 0;
}


void saveSensorData(void)
{
  int index=0;
  #if DEBUG_PRINTS
  unsigned long timeUs;
  timeUs = micros();
  //Serial.println("timer start");
  #endif
  interrupts();
  for (index = 0; index< NUM_OF_SENSORS; index++)
  {
    sensorOutputData[index].mV = PS_ReadSensor(index);
    sensorOutputData[index].unitX10 = getSensorUnitsx10(index, sensorOutputData[index].mV);
  }
  //interrupts();
  #if DEBUG_PRINTS
  timeUs = micros()-timeUs;
  Serial.print("done in ");Serial.println(timeUs);
  #endif
  gSensorDataUpdated = 1;
}

#define MAX_PS2_SAMPLES 10
#define THRESHOLD_COMPARE_INDEX 2
#define DIP_THRESHOLD 5 //better to be lower than PEEP

static int ps2Samples[MAX_PS2_SAMPLES];
static int diffArray[MAX_PS2_SAMPLES];
static int ps2SamplesIndex = 0;
boolean checkForPs2Dip()
{
  int previousIndex;
  boolean dipDetected = false;

  /*
   * update data in buffer
   */
   ps2Samples[ps2SamplesIndex] = sensorOutputData[PS2].unitX10;

   /*
    * check with previous samples
    */
   for (int index = 1; index < MAX_PS2_SAMPLES; index++)
   {
     previousIndex = ps2SamplesIndex - index;
     if (previousIndex < 0)
     {
       previousIndex += MAX_PS2_SAMPLES;
     }
     diffArray[index] = ps2Samples[previousIndex] - ps2Samples[ps2SamplesIndex];
   }

   /*
    * compare against threshold
    */
   if (diffArray[THRESHOLD_COMPARE_INDEX] < DIP_THRESHOLD)
   {
     dipDetected = true;
   }
   
   /*
    * increment index and be ready for next cycle.
    */
   ps2SamplesIndex++;
   if (ps2SamplesIndex >= MAX_PS2_SAMPLES);
   {
     ps2SamplesIndex = 0;
   }
}
