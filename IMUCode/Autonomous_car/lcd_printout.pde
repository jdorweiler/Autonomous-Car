// Prints data to the LCD screen
void lcdPrint() {
  Serial.println("Start LCD Print");
  lcd.setCursor(0, 0);
  lcd.print(localData[5]); // Kalman heading
  lcd.setCursor(9, 0);
  lcd.print(localData[6]);  // Heading to waypoint
  lcd.setCursor(0, 1);
  lcd.print(localData[8]); // angleDiff sent to PID
  lcd.setCursor(9,1);
  lcd.print(localData[7]); // Distance in meters
 /* 
          serialFloatPrint(localData[5]);
          serialFloatPrint(localData[8]);
          serialFloatPrint(localData[7]);
  */        
  return;
}
// Send data to GUI through the Xbee 
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  Serial.print("f:");
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);
/*  // DEBUG 
  Serial.println();
  Serial.print(b[0],BIN);
  Serial.print(b[1], BIN);
  Serial.print(b[2], BIN);
  Serial.println(b[3], BIN);*/
  
  
  /*
  for ( int i = 0; i < 12; i = i + 1) {
          Serial.print(localData[i]); Serial.print(" ");
           } 
           Serial.println();
*/
}
