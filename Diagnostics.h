
#include "pinout.h"

typedef enum 
{
  RT_INC=0,
  RT_DEC,
  RT_BT_PRESS,
  RT_NONE
}RT_Events_T;

void Diagnostics_Mode(void);
RT_Events_T Encoder_Scan(void);

unsigned long lastButtonPress = 0;
int currentStateCLK;
int lastStateCLK;

void Diagnostics_Mode(void)
{
  RT_Events_T eRTState = RT_NONE;
  eRTState = Encoder_Scan();
  //switch(eRTState)
  
}


RT_Events_T Encoder_Scan(void)
{
  RT_Events_T eRTState = RT_NONE;
  
  // Read the current state of CLK
  currentStateCLK = digitalRead(DISP_ENC_CLK);
  
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if(currentStateCLK != lastStateCLK)
  {
   // delay(1);
    currentStateCLK = digitalRead(DISP_ENC_CLK);
    if((currentStateCLK != lastStateCLK) && (currentStateCLK == 1))
    {
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(DISP_ENC_DT) != currentStateCLK) {
        eRTState = RT_DEC;
      } else {
        // Encoder is rotating CW so increment
        eRTState = RT_INC;
      }
    }
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(DISP_ENC_SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) 
  {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      eRTState = RT_BT_PRESS;
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  //delay(1);
  return(eRTState);
}
