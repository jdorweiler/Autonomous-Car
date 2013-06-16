void startup() {
  Serial.println("Start Up");
    // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);  
//*************************************  Waypoints        ****************************  
/* Fubar parking lot
waypoints[0][0]=40.506125;  waypoints[0][1]=-74.432363;  
waypoints[1][0]=40.506326;  waypoints[1][1]=-74.43267;  
waypoints[2][0]=40.506237;  waypoints[2][1]=-74.432766;  
waypoints[3][0]=40.50614;  waypoints[3][1]=-74.432341;   
waypoints[4][0]=40.505883;   waypoints[4][1]=-74.432206;  
*/

// school by house
waypoints[0][0]=40.610524;  waypoints[0][1]=-74.876335;  
waypoints[1][0]=40.610807;  waypoints[1][1]=-74.876402;  
waypoints[2][0]=40.610811;  waypoints[2][1]=-74.876714;  

//***********************************************************************88
//pinMode(7,INPUT); //endoder interrupt on pin 7

// LCD Startup screen  
  lcd.begin(16, 2);
  lcd.print("Autonomous Car ");
  delay(500);
  lcd.clear();
  while (!SD.begin(chipSelect)) {
    lcd.print("Check SD Card :(");
    Serial.println("SD CARD");
    //return;
  }
 Serial.println("card initialized.");
   
  setupgyroscope(2000); // Configure to  - 250, 500 or 2000 deg/sec  

//Set up parameters for GPS
  Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  Serial1.println(PMTK_SET_NMEA_UPDATE_5HZ);
  
// Servos and start up steering check      
  ser_esc.attach(25);
  ser_steer.attach(39);
  delay(500); //wait for the sensors to be ready 
  ser_steer.write(1500); delay(100);
  ser_steer.write(1200); delay(100);
  ser_steer.write(1500); delay(100);
  ser_steer.write(1800); delay(100);
  ser_steer.write(1500); ser_esc.write(1560);
  
 /* 
// Line marker for SD files.  Marks the start of a new run  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("******************************************************************** ");
  dataFile.close(); 
  pinMode(53, OUTPUT); 
 */
// Sit and wait for GPS signal before we do anything.  Print to LCD 
  int i = 0;
  while (!feedgps()){
    lcd.setCursor(0,0);
    lcd.print("Waiting for GPS");
    lcd.setCursor(i,1);
    lcd.print(".");
    i = i++;
    delay(1000);
  }
  lcd.clear();
  
// This prints out data to the LCD and waits for a button push to start the car.  The idea is to give it a bit 
// of time to collect so GPS data. 
 while(!buttonState == HIGH){
    if (Serial.available()) { // If data is available to read,
    start = Serial.read(); // read it and store it in val
       if (start == 'H'){
           break; 
       }
    }
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {start = 'H';}
    if (feedgps()){
      gpsdump(gps,waypoints[0][0],waypoints[0][1]);
    lcd.setCursor(0,0);
    lcd.print("Press Button for");
    lcd.setCursor(0,1);
    lcd.print("Auto Driving");
    }
    delay(100);
  }
// Reset to first waypoint just in case  
  num = 0;
  
// Clear LCD to get ready for angle/distance data  
lcd.clear();
}


  

