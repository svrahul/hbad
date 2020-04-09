#include <LiquidCrystal.h>

#define LCD_LENGTH_CHAR 20
#define LCD_HEIGHT_CHAR 4
char paddedValue[10];
LiquidCrystal lcd(DISPLAY_RS_PIN, DISPLAY_EN_PIN, DISPLAY_4_PIN, DISPLAY_3_PIN, DISPLAY_2_PIN, DISPLAY_1_PIN);

void cleanRow(unsigned short row) {
  lcd.setCursor(0, row);
  for (int i = 0; i < LCD_LENGTH_CHAR; i++) {
    lcd.print(" ");
  }
}
