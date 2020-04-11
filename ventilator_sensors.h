
#include <Wire.h>
#include "./libraries/Adafruit_ADS1X15/Adafruit_ADS1015.h"
#include "./libraries/Adafruit_ADS1X15/Adafruit_ADS1015.cpp"

Adafruit_ADS1115 ads; 


// These constants won't change. They're used to give names to the pins used:
const int OxygenAnalog = A0;  // Analog input pin that the potentiometer is attached to
const int PotValue = A1;  // Analog input pin that the potentiometer is attached to
const int MaxAdcSamples = 5;
int ADCSampleBuff[MaxAdcSamples]= {0};

float OxygenFianlAnalog;
const int ADCRefVolt = 5;
const float  O2SensMultiplier= 0.00488;
const float  PressSensMultiplier= 0.0001875;

float O2SensorVolt;
float PressSensVolt;

int O2Percentage;
float Pressure;

/*Debug prints enable */
#define SERIAL_PRINTS 0

#define OXYGEN_ANALOG_PIN   A0
#define OXYGEN_SET_ANA_PIN  A1

/*ADS115 ADC Channel Assignments */
//#define PS1 0
//#define PS2 1
//#define DPS1  2
//#define DPS2  3

#define NUM_OF_SENSORS 5
typedef enum{
  PS1,
  PS2,
  DPS1,
  DPS2,
  O2,
}sensor_e;

float ADC_ReadVolageOnATMega2560(int Channel);
float ADS1115_ReadVolageOverI2C(int Channel);
void ADS1115_init(void);
float ADC_ApplyAvgFilter(int *SampleBuf, int SampleCount, float Multiplier);
int ADC_GetMedian(int *SampleBuf, int len);
// the loop function runs over and over again forever
/*void test_loop() {
  #if SERIAL_PRINTS
  Serial.println("Hello World..!!");
  #endif
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  O2SensorVolt = ADC_ReadVolageOnATMega2560(A0);
  #if SERIAL_PRINTS
  Serial.print("Oxygen(%) = ");
  Serial.println(O2Percentage);
  #endif
  
 
  PressSensVolt = ADS1115_ReadVolageOverI2C(DPS2);  
  #if SERIAL_PRINTS
  Serial.print("Pressure in cmXX = ");
  Serial.println(Pressure);
  #endif
}
*/
void ADS1115_init(void)
{
ads.begin();
}



float ADS1115_ReadVolageOverI2C(int Channel)
{
  int ADCCount, AvgSampleCount;
  float Avg10Samples;
  float SumValue= 0.0, ADCThresH=0.0, ADCThresL=0.0;
  float PressSensorVolts=0.0;
  for(int i=0; i<MaxAdcSamples; i++)
  {
    ads.readADC_SingleEnded(Channel);
    while(digitalRead(ADS115_INT_PIN));
    ADCSampleBuff[i] = ads.readADC_ConvertedSample();
    #if SERIAL_PRINTS
    Serial.print(ADCSampleBuff[i]);
    Serial.print(" ");
    #endif
  }
  PressSensorVolts = ADC_ApplyAvgFilter(ADCSampleBuff, MaxAdcSamples, PressSensMultiplier);
  #if SERIAL_PRINTS
  Serial.print("Press Sensor ");
  Serial.println(Channel); 
  Serial.println(PressSensorVolts);
  #endif

  return(PressSensorVolts);
}
  
float ADC_ReadVolageOnATMega2560(int Channel)
{
  
  int ADCCount, AvgSampleCount;
  float Avg10Samples;
  float SumValue= 0.0, ADCThresH=0.0, ADCThresL=0.0;
  float OxygenSensorVolts=0.0;
  for(int i=0; i<MaxAdcSamples; i++)
  {
    ADCSampleBuff[i] = analogRead(Channel);
    #if SERIAL_PRINTS
    Serial.print(ADCSampleBuff[i]);
    Serial.print(" ");
    #endif
  }
  OxygenSensorVolts = ADC_ApplyAvgFilter(ADCSampleBuff, MaxAdcSamples, O2SensMultiplier);
  #if SERIAL_PRINTS
  Serial.print("Oxygen ADC Value= ");
  Serial.println(OxygenSensorVolts);
  #endif

  return(OxygenSensorVolts);
}

float ADC_ApplyAvgFilter(int *SampleBuf, int SampleCount, float Multiplier)
{
  int MedianSample;
 
  MedianSample = ADC_GetMedian(SampleBuf,SampleCount);
 #if SERIAL_PRINTS
 Serial.print("Median ");
 Serial.println(MedianSample);
 
  for(int i=0; i<SampleCount; i++)
  {
    Serial.print(SampleBuf[i]);
    Serial.print(" ");
   }
  #endif
  return(MedianSample * Multiplier);
  
}

int ADC_GetMedian(int *SampleBuf, int len)
{
  int i,j,a;
  long int median = 0;
  //sort the samples 
  for (i = 0; i < len; ++i) 
  {
    for (j = i + 1; j < len; ++j)
    {
      if (SampleBuf[i] > SampleBuf[j]) 
      {
        a =  SampleBuf[i];
        SampleBuf[i] = SampleBuf[j];
        SampleBuf[j] = a;
      }
    }
  }

  i = len/2;
  if((len%2)==0)
  {
    median = ((long int)SampleBuf[i-1]+(long int)SampleBuf[i])/2;
  }
  else
  {
    median = SampleBuf[i];
  }
  return((int)median);
}

 /*
float ADC_ApplyAvgFilter(int *SampleBuf, int SampleCount, float Multiplier)
{
  int AvgSampleCount;
  float Avg10Samples;
  float SumValue= 0.0, ADCThresH=0.0, ADCThresL=0.0;
  float SensorVolts=0.0;
  float ADCAnalogValues[SampleCount] = {0.0};
  
  for(int i=0; i<SampleCount; i++)
  {
    ADCAnalogValues[i] = SampleBuf[i]*Multiplier;
    #if SERIAL_PRINTS
    Serial.print(ADCAnalogValues[i]);
    Serial.print(" ");
    #endif
    SumValue+=ADCAnalogValues[i];
  }

  Avg10Samples = SumValue/SampleCount;
  #if SERIAL_PRINTS
   Serial.print("Avg 10 samples = ");
   Serial.println(Avg10Samples);
   #endif
  ADCThresH = Avg10Samples + Multiplier;
  ADCThresL = Avg10Samples - Multiplier;

  AvgSampleCount = 0;
  SumValue=0.0;
  for(int i=0; i<SampleCount; i++)
  {
    if((ADCAnalogValues[i]>= ADCThresL) && (ADCAnalogValues[i] <= ADCThresH))
    {
      AvgSampleCount++;
      SumValue+=ADCAnalogValues[i];
      #if SERIAL_PRINTS
      Serial.print(ADCAnalogValues[i]);
      Serial.print(" ");
      #endif
    }
    if(AvgSampleCount == 0)
    {
      SensorVolts = Avg10Samples;
    }
    else
    {
      SensorVolts = SumValue/AvgSampleCount;
    }
  }
  
  return (SensorVolts); 
}
*/
