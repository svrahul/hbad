
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;


// These constants won't change. They're used to give names to the pins used:
const int OxygenAnalog = A0;  // Analog input pin that the potentiometer is attached to
const int PotValue = A1;  // Analog input pin that the potentiometer is attached to
const int MaxAdcSamples = 10;

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
#define PS1 0
#define PS2 1
#define DPS1  2
#define DPS2  3


float ADC_ReadVolageOnATMega2560(int Channel);
float ADS1115_ReadVolageOverI2C(int Channel);
void ADS1115_init(void);
float ADC_ApplyAvgFilter(int *SampleBuf, int SampleCount, float Multiplier);

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
  int ADCSampleBuff[MaxAdcSamples]= {0};
  float Avg10Samples;
  float SumValue= 0.0, ADCThresH=0.0, ADCThresL=0.0;
  float PressSensorVolts=0.0;
  for(int i=0; i<MaxAdcSamples; i++)
  {
    ADCSampleBuff[i] = ads.readADC_SingleEnded(Channel);
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
  int ADCSampleBuff[MaxAdcSamples]= {0};
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

float ReadOxygenSensorAnalogVolts(void)
{
  int ADCCount, AvgSampleCount;
  int ADCSampleBuff[MaxAdcSamples]= {0};
  float Avg10Samples;
  float SumValue= 0.0, ADCThresH=0.0, ADCThresL=0.0;
  float OxygenSensorVolts=0.0;
  for(int i=0; i<MaxAdcSamples; i++)
  {
    ADCSampleBuff[i] = analogRead(OxygenAnalog);
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
