
#include "lcd.h"
#define SERIAL_PRINTS 0

typedef enum 
{
  RT_INC=0,
  RT_DEC,
  RT_BT_PRESS,
  RT_NONE
}RT_Events_T;

typedef enum{
  mainMenuE,
  subMenu1E,
  subMenu2E,
  subMenu3E,
  MAX_MENUS,
}menuIndex;

typedef enum{
  MENU_LEVEL_0,
  MENU_LEVEL_1,
  MAX_LEVEL,
}menuLevelT;

typedef struct {
  char * menuName;
  char ** menu;
  int menuLength;
  menuLevelT menuLevel;
  void (* functionPtr)();
}menuItemsT;

void Diagnostics_Mode(void);
RT_Events_T Encoder_Scan(void);

unsigned long lastButtonPress = 0;
int currentStateCLK;
int lastStateCLK;


menuIndex currentMenuIdx = mainMenuE;
int currentMenuLevel = 0;
int CursorLine = 0;
int DisplayFirstLine = 0;
int seletIndicator = 1; // can be 1,2,3
int scrollIndex = 0;


#define MAIN_MENU_LENGTH 4
#define SUB_MENU1_LENGTH 6
#define SUB_MENU2_LENGTH 2
#define SUB_MENU3_LENGTH 4




char* mainMenu[MAIN_MENU_LENGTH] = {" exit diag mode", " O2-Calib"," Check ADS1115"," Read SOL"};
char* subMenu1[SUB_MENU1_LENGTH] = {" go back", " O2 0%"," O2 21.6%"," O2 28%", " O2 40%"," O2 100%"};
char* subMenu2[SUB_MENU2_LENGTH] = {" go back", " validate ADC"};
char* subMenu3[SUB_MENU3_LENGTH] = {" go back", " Read SOL1"," Read SOL2"," Read SOL3"};

void diagO2Sensor(void);
void diagAds1115(void);
void diagSolStatus(void);
void setup_service_mode ();
void print_menu_common( menuIndex menuIdx);
void displayEditMenu(void);
RT_Events_T encoderScanUnblocked();

menuItemsT menuItems[MAX_MENUS] = 
{
  [mainMenuE] = {"Main menu ", mainMenu, MAIN_MENU_LENGTH, MENU_LEVEL_0, NULL},
  [subMenu1E] = {"SubMenu1 O2 ", subMenu1, SUB_MENU1_LENGTH, MENU_LEVEL_1, &diagO2Sensor},
  [subMenu2E] = {"SubMenu2 PS ", subMenu2, SUB_MENU2_LENGTH, MENU_LEVEL_1, &diagAds1115},  
  [subMenu3E] = {"SubMenu3 SOL ", subMenu3, SUB_MENU3_LENGTH, MENU_LEVEL_1, &diagSolStatus}
};


void setup_service_mode ()
{
//  Serial.begin(9600);
  digitalWrite (DISP_ENC_DT, HIGH);     // enable pull-ups
  digitalWrite (DISP_ENC_CLK, HIGH);
  digitalWrite (DISP_ENC_SW, HIGH); 
//
//
//  lcd.begin (20,4);

}  // end of setup

void loop__ ()
{
     //Diagnostics_Mode();
}  


boolean no_input = true;
RT_Events_T encoderScanUnblocked()
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
        no_input = false;
      } else {
        // Encoder is rotating CW so increment
        eRTState = RT_INC;
        no_input = false;
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
      no_input = false;
    }

    // Remember last button press event
    lastButtonPress = millis();
  }
  if (eRTState != RT_NONE)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1);
    digitalWrite(BUZZER_PIN, LOW);
  }
  return eRTState;
}

int Menu_Sel=0;

boolean continue_diag_mode = true;
RT_Events_T Encoder_Scan(void)
{
  RT_Events_T eRTState = RT_NONE;
  no_input = true;
  while(no_input)
  {
    eRTState = encoderScanUnblocked();
  }
  // Put in a slight delay to help debounce the reading
  //delay(1);
  return(eRTState);
}



void print_menu_common( menuIndex menuIdx)
{
  int numOfRowsToWrite=0;
  String strOnLine234 = ">";
  #if SERIAL_PRINTS
//  Serial.println("in print_menu_common menu, slect, scroll");
//  Serial.println(menuIdx);
//  Serial.println(seletIndicator);
//  Serial.println(scrollIndex);
  #endif
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuItems[menuIdx].menuName);
  #if SERIAL_PRINTS
//  Serial.println(menuItems[menuIdx].menuName);
  #endif
  numOfRowsToWrite = min(LCD_HEIGHT_CHAR-1, menuItems[menuIdx].menuLength);
  for (int i=0; i < numOfRowsToWrite; i++)//menuItems[menuIdx].menuLength; i++)
  {
    lcd.setCursor(0,i+1);
    if (seletIndicator == i+1)
    {
      strOnLine234 = ">";
    }
    else
    {
      strOnLine234 = " ";
    }
      strOnLine234 += menuItems[menuIdx].menu[scrollIndex + i];
      lcd.print (strOnLine234);
      #if SERIAL_PRINTS
//      Serial.println(strOnLine234);
      #endif
  }
  lcd.setCursor(0,(CursorLine-scrollIndex)+1);
}


void move_up()
{
  if (menuItems[currentMenuIdx].menuLength <= LCD_HEIGHT_CHAR-1) //menu length starts with 1
  {
    scrollIndex = 0;
  }
  seletIndicator++;
  /*
   * check wrap around of select indicator
   */
  if ((seletIndicator > menuItems[currentMenuIdx].menuLength)||
      (seletIndicator > LCD_HEIGHT_CHAR-1))
  {
    seletIndicator=min(LCD_HEIGHT_CHAR-1,menuItems[currentMenuIdx].menuLength);
    if (seletIndicator == LCD_HEIGHT_CHAR-1)
    {
      scrollIndex++;
    }
    if ((scrollIndex + seletIndicator) > menuItems[currentMenuIdx].menuLength)
    {
      scrollIndex = menuItems[currentMenuIdx].menuLength - seletIndicator;
    }
  }
  print_menu_common (currentMenuIdx);
}

void move_down()
{
  if (menuItems[currentMenuIdx].menuLength <= LCD_HEIGHT_CHAR-1) //menu length starts with 1
  {
    scrollIndex = 0;
  }
  seletIndicator--;
  /*
   * check wrap around of select indicator
   */
  if (seletIndicator == 0 )
  {
    seletIndicator=1;
    if (scrollIndex>0)
    {
      scrollIndex--;
    }
  }
  print_menu_common (currentMenuIdx);
}

void selection()
{

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("You selected:");
  lcd.setCursor(0,2);
  lcd.print(menuItems[currentMenuIdx].menu[seletIndicator + scrollIndex -1 ]);
  delay(2000);
  lcd.clear();

  if ((seletIndicator == 1) &&
      (scrollIndex == 0))
  {
    if (currentMenuIdx != mainMenuE)
    {
      currentMenuLevel = MENU_LEVEL_0; 
      currentMenuIdx = mainMenuE;
    }
    else
    {
      continue_diag_mode = false;
    }
  }
  else
  {
    currentMenuLevel++;
    
    if (currentMenuLevel >= MAX_LEVEL )
    {
      if (menuItems[currentMenuIdx].functionPtr != NULL)
      {
        (menuItems[currentMenuIdx].functionPtr)();
      }
    }
    currentMenuIdx = seletIndicator + scrollIndex-1;
    if (currentMenuLevel >= MAX_LEVEL )
    {
      currentMenuLevel = MENU_LEVEL_0; 
      currentMenuIdx = mainMenuE;
    }
  }
  seletIndicator=1;
  scrollIndex=0;
  print_menu_common(currentMenuIdx);
}



void Diagnostics_Mode(void)
{
//  Serial.println("in Diagnostics_Mode");
  while(continue_diag_mode)
  {
    RT_Events_T eRTState = RT_NONE;
    if (!Menu_Sel)
    {
      print_menu_common(mainMenuE);
      Encoder_Scan();
      Menu_Sel++;
    }
    eRTState = Encoder_Scan();
    #if SERIAL_PRINTS
//    Serial.print("in Diagnostics_Mode");Serial.println(eRTState);
    #endif
    switch(eRTState)
    {
      case RT_INC:
         move_up();
         break;
      case   RT_DEC:
         move_down();
         break;
      case   RT_BT_PRESS:
         selection();
         break;
    }
  }
}

int initSelect = 0;
void move_up_init()
{
  lcd.setCursor(0,1);
  lcd.print(">>Edit parameters");
  lcd.setCursor(0,2);
  lcd.print("  Enter Diagnostics");
  initSelect = 1;
}

void move_down_init()
{
  lcd.setCursor(0,1);
  lcd.print("  Edit parameters");
  lcd.setCursor(0,2);
  lcd.print(">>Enter Diagnostics");
  initSelect = 2;
}

void selection_init()
{
  if  (initSelect == 1)
  {
    displayEditMenu();
  }
  else if (initSelect == 2)
  {
    Diagnostics_Mode();
  }
  initSelect = 0;
}

void displayInitialScreen()
{
  boolean continueLoop = true;
  int wait = 299;
  RT_Events_T eRTState = RT_NONE;
  encoderScanUnblocked();
  encoderScanUnblocked();
  encoderScanUnblocked();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Rotate encoder");
  lcd.setCursor(0,1);
  lcd.print("  Edit parameters");
  lcd.setCursor(0,2);
  lcd.print("  Enter Diagnostics");
  while( wait>0 )
  {
    eRTState = encoderScanUnblocked();
    if (eRTState != RT_NONE)
    {
      break;
    }
    lcd.setCursor(0,3);
    lcd.print(wait/100);
    delay (10);
    wait--;
  }
  if (eRTState != RT_NONE)
  {
    while (continueLoop)
    {
      eRTState = Encoder_Scan();
      #if SERIAL_PRINTS
//      Serial.print("in displayInitialScreen input received");Serial.println(eRTState);
      #endif
      switch(eRTState)
      {
        case RT_INC:
           move_up_init();
           break;
        case   RT_DEC:
           move_down_init();
           break;
        case   RT_BT_PRESS:
           selection_init();
           continueLoop = false;
           break;
      }
    }
  }
//  Serial.println("exited displayInitialScreen");
}
