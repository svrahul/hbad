#include <LiquidCrystal.h>

#define LCD_LENGTH_CHAR 20
#define LCD_HEIGHT_CHAR 4
#define NAME1_DISPLAY_POS 0
#define VALUE1_DISPLAY_POS 5
#define NAME2_DISPLAY_POS 11
#define VALUE2_DISPLAY_POS 14
#define PAR_SEP_CHAR ' '
char paddedValue[10];
LiquidCrystal lcd(DISPLAY_RS_PIN, DISPLAY_EN_PIN, DISPLAY_4_PIN, DISPLAY_3_PIN, DISPLAY_2_PIN, DISPLAY_1_PIN);

void cleanRow(unsigned short row) {
  lcd.setCursor(0, row);
  for (int i = 0; i < LCD_LENGTH_CHAR; i++) {
    lcd.print(" ");
  }
}


void cleanColRow(unsigned short col, unsigned short row) {
  lcd.setCursor(col, row);
  for (int i = col; i < LCD_LENGTH_CHAR; i++) {
    lcd.print(" ");
  }
}

void printPadded(int unpaddedNumber) {
  if (unpaddedNumber < 10) {
    lcd.print("  ");
  }else if (unpaddedNumber < 100) {
    lcd.print(" ");
  }
  lcd.print(unpaddedNumber);

}
