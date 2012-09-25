// Prints data to the LCD screen
void lcdPrint() {
  lcd.setCursor(0, 0);
  lcd.print(localData[5]); // Kalman heading
  lcd.setCursor(9, 0);
  lcd.print(localData[6]);  // Heading to waypoint
  lcd.setCursor(0, 1);
  lcd.print(localData[8]); // angleDiff sent to PID
  lcd.setCursor(9,1);
  lcd.print(localData[7]); // Distance in meters
  return;
}

