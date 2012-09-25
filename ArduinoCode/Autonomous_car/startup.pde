void startup() {
    // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);  
//*****************************************************************  
waypoints[0][0]=40.505589;  waypoints[0][1]=-74.433531;  
waypoints[1][0]=40.505658;  waypoints[1][1]=-74.433906;  
waypoints[2][0]=40.50552;  waypoints[2][1]=-74.433721;  
waypoints[3][0]=40.505794;  waypoints[3][1]=-74.433838;  
waypoints[4][0]=40.505565;   waypoints[4][1]=-74.433564;  



//***********************************************************************88


// LCD Startup screen  
  lcd.begin(16, 2);
  lcd.print("Autonomous Car ");
  delay(1000);
  lcd.clear();
  if (!SD.begin(chipSelect)) {
    lcd.print("SD card failed");
    return;
  }
 // Serial.println("card initialized.");
  
//  Serial.println("Turning on the accelerometer");
  writeTo(ACC, 0x2D, 0);      
  writeTo(ACC, 0x2D, 16);
  writeTo(ACC, 0x2D, 8); 
  setupgyroscope(2000); // Configure to  - 250, 500 or 2000 deg/sec  

//Set up parameters for GPS
  Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  Serial1.println(PMTK_SET_NMEA_UPDATE_5HZ);

// Line marker for SD files  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("******************************************************************** ");
  dataFile.close(); 
  pinMode(53, OUTPUT); 

// Servos and start up steering check      
  ser_esc.attach(8);
  ser_steer.attach(9);
  delay(2500); //wait for the sensors to be ready 
  ser_steer.write(1500); delay(100);
  ser_steer.write(1200); delay(100);
  ser_steer.write(1500); delay(100);
  ser_steer.write(1800); delay(100);
  ser_steer.write(1500); ser_esc.write(1500);
   
// Wait for GPS signal to do anything 
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
  
// This prints out data to the LCD and waits for a button push to start the car.  This way I can be sure
// it starts out with good data
  while(!buttonState == HIGH){
    buttonState = digitalRead(buttonPin);
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
  
// Clear LCD for angle/distance data  
lcd.clear();
}


  

