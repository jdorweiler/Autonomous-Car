
void lcdPrint() {
  lcd.setCursor(0, 0);
  lcd.print(localData[2]); // Kalman heading
  lcd.setCursor(9, 0);
  lcd.print(localData[3]);  // Heading to waypoint
  lcd.setCursor(0, 1);
  lcd.print(localData[9]); // angleDiff sent to PID
  lcd.setCursor(9,1);
  lcd.print(localData[4]); // Distance in meters
}

