
#include <Wire.h>
#include "./libraries/Adafruit_ADS1X15/Adafruit_ADS1015.h"
#include "./libraries/Adafruit_ADS1X15/Adafruit_ADS1015.cpp"

Adafruit_ADS1115 ads; 


// These constants won't change. They're used to give names to the pins used:
const int OxygenAnalog = A0;  // Analog input pin that the potentiometer is attached to
const int PotValue = A1;  // Analog input pin that the potentiometer is attached to
const int MaxAdcSamples = 5;
int ADCSampleBuff[MaxAdcSamples] = {0};

#define VIntRef 1100u //1.1V Internal Ref
#define AVCC_DYNAMIC 0
#define MEDIAN_ADC_FILTER 1


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

#define OXYGEN_ANALOG_PIN   A4
#define OXYGEN_SET_ANA_PIN  A1

/*ADS115 ADC Channel Assignments */
//#define PS1 0
//#define PS2 1
//#define DPS1  2
//#define DPS2  3

typedef enum{
  PS1,
  PS2,
  DPS1,
  DPS2,
  O2,
  NUM_OF_SENSORS,
}sensor_e;

float ADC_ReadVolageOnATMega2560(int Channel);
float ADS1115_ReadVolageOverI2C(int Channel);
void ADS1115_init(void);
float ADC_ApplyAvgFilter(int *SampleBuf, int SampleCount, float Multiplier);
int ADC_GetMedian(int *SampleBuf, int len);
#if AVCC_DYNAMIC
int getVrefVoltage(void);
#endif


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
  float SumValue = 0.0, ADCThresH = 0.0, ADCThresL = 0.0;
  float OxygenSensorVolts = 0.0;
#if AVCC_DYNAMIC
  int Vref = 0;
  float DynacmicO2SensMult = 0.0;
  Vref = getVrefVoltage();
  //Serial.print(" Vref Volt = ");
  /// Serial.print(Vref);
  DynacmicO2SensMult = Vref;
  DynacmicO2SensMult /= 1024;
  DynacmicO2SensMult /= 1000; //Coversion of mV to V

  // Serial.print(" DynacmicO2SensMult = ");
  // Serial.println(DynacmicO2SensMult);

#endif

  for (int i = 0; i < MaxAdcSamples; i++)
  {
    ADCSampleBuff[i] = analogRead(Channel);
    #if SERIAL_PRINTS
    Serial.print(ADCSampleBuff[i]);
    Serial.print(" ");
    #endif
  }

#if AVCC_DYNAMIC
  OxygenSensorVolts = ADC_ApplyAvgFilter(ADCSampleBuff, MaxAdcSamples, DynacmicO2SensMult);
#else
  OxygenSensorVolts = ADC_ApplyAvgFilter(ADCSampleBuff, MaxAdcSamples, O2SensMultiplier);
#endif

  #if SERIAL_PRINTS
  Serial.print("Oxygen ADC Value= ");
  Serial.println(OxygenSensorVolts);
  #endif

  return(OxygenSensorVolts);
}
#if AVCC_DYNAMIC
int getVrefVoltage(void)
{
  int ADCSample[MaxAdcSamples] = {0};
  int results, ADCCount;
  const long InternalReferenceVoltage = 1100L;  // Adust this value to your specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref.
  // MUX3 MUX2 MUX1 MUX0  --> 1 1110 1.1V (VBG)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  ADCSRA |= ADEN;
  delay(5);
  for (int i = 0; i < MaxAdcSamples; i++)
  {
    // Start a conversion
    ADCSRA |= _BV( ADSC );
    // Wait for it to complete
    while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
    ADCSample[i] = ADC;
  }
  ADCCount = ADC_GetMedian(ADCSample, MaxAdcSamples);
  // Scale the value
  results = ((InternalReferenceVoltage * 1024L) / ADCCount);
  ADCSRA &= (~ADEN);
  return results;
}
#endif

#if MEDIAN_ADC_FILTER
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
  return (MedianSample * Multiplier);
}
#else
float ADC_ApplyAvgFilter(int *SampleBuf, int SampleCount, float Multiplier)
{
  int AvgSampleCount;
  float Avg10Samples;
  float SumValue = 0.0, ADCThresH = 0.0, ADCThresL = 0.0;
  float SensorVolts = 0.0;
  float ADCAnalogValues[SampleCount] = {0.0};

  for (int i = 0; i < SampleCount; i++)
  {
    ADCAnalogValues[i] = SampleBuf[i] * Multiplier;
    #if SERIAL_PRINTS
    Serial.print(ADCAnalogValues[i]);
    Serial.print(" ");
    #endif
    SumValue += ADCAnalogValues[i];
  }

  Avg10Samples = SumValue / SampleCount;
  #if SERIAL_PRINTS
  Serial.print("Avg 10 samples = ");
  Serial.println(Avg10Samples);
  #endif
  ADCThresH = Avg10Samples + Multiplier;
  ADCThresL = Avg10Samples - Multiplier;

  AvgSampleCount = 0;
  SumValue = 0.0;
  for (int i = 0; i < SampleCount; i++)
  {
    if ((ADCAnalogValues[i] >= ADCThresL) && (ADCAnalogValues[i] <= ADCThresH))
    {
      AvgSampleCount++;
      SumValue += ADCAnalogValues[i];
      #if SERIAL_PRINTS
      Serial.print(ADCAnalogValues[i]);
      Serial.print(" ");
      #endif
    }
    if (AvgSampleCount == 0)
    {
      SensorVolts = Avg10Samples;
    }
    else
    {
      SensorVolts = SumValue / AvgSampleCount;
    }
  }

  return (SensorVolts);
}
#endif
#if (MEDIAN_ADC_FILTER | AVCC_DYNAMIC)
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
#endif
