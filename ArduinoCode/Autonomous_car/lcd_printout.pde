
void lcdPrint() {
  lcd.setCursor(0, 0);
lcd.print(localData[2]); 
lcd.setCursor(9, 0);
lcd.print(localData[3]);
lcd.setCursor(0, 1);
lcd.print(localData[9]); 
lcd.setCursor(9,1);
lcd.print(localData[4]/10);
}

