#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS.h>
#include <SD.h>
#include <LiquidCrystal.h>

// Start Button
const int buttonPin = 83;
int buttonState = 0; 

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// CS pin for SD logging
const int chipSelect = 53;



// Pots for manual adjustments//////////////////////////////////////////////////////////////
int sensorPin1 = A8;
int sensorPin2 = A9;
int sensorPin3 = A10;
float sensorValue1 = 0;
float sensorValue2 = 0;
float sensorValue3 = 0;
////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////// GPS  ///////////////////////////////////////////////////////////////////
TinyGPS gps;
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define TO_READ (6)      //num of bytes to read each time (two bytes for each axis)

static void gpsdump(TinyGPS &gps, float LONDON_LAT, float LONDON_LON);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);
float dist;
int num; //waypoint number
float WayPT_Lat,WayPT_Lon;
bool newGPSheading;
unsigned long startDt;
/////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// PID & Motion  ////////////////////////////////////////////////
//Point where it switches from conservative to agressive 
int gapDist=3;
//Aggressive
double aggK=0.9, aggKp=7, aggKi=4, aggKd=1; 
//Conservative
double consK=0.6, consKp=3, consKi=2, consKd=0.5;
int steering;
Servo ser_esc;
Servo ser_steer;
//////////////////////////////////////////////////////////////////////////////////////////////////



/////////////// Accelerometer and Gyroscope////////////////////////////////////////////////////////////
int ACC_angle;
float GYRO_rate;
float angleDiff, currAngle, actAngle;
byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];          //string buffer to transform data before sending it to the serial port
float GYR_Y, ACC_X, ACC_Y  , ACC_Z; 
int gyroscope_Address = 105; //I2C address of the gyroscope
#define ACC (0x53)    //accelerometer address
#define TO_READ (6)      //num of bytes to read each time (two bytes for each axis)
#define   LINE_END              10                             // \n
#define   SPLIT                 58                             // :
#define CTRL_REG1 0x20 // gyroscope stuff
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

unsigned long pastMicros = 0;
unsigned long currMicros = 0;
unsigned long currMillis = 0;
unsigned long pastMillis = 0;
int gyroHigh, gyroLow;
float degreesPerSecond;
double dt = 0.0;
float gyroAngle;
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////// Timing for Kalman Filter //////////////////////////////////////////////////////////
int  STD_LOOP_TIME  = 10;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////  Logging and Waypoints ///////////////////////////////////////////////////////////////
double localData[20]; // Array holding localication data {lat,lon,wayptLat,wayptLon,currAngle,KalmanAngle,Setpoint,dist}
float gyroAngles[2];  // Array holding gyro angle [0] and gyro rate [1] in degrees and degrees/ms
float waypoints[30][2];// Array holding the waypoint coordinates
////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(57600);
  Serial1.begin(9600);
  Wire.begin();
     
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);  
//*****************************************************************  
waypoints[0][0]=40.5869785;  waypoints[0][1]=-74.8901288;  
waypoints[1][0]=40.5867896;  waypoints[1][1]=-74.8900319;  
waypoints[2][0]=40.5866876;  waypoints[2][1]=-74.889993;  
waypoints[3][0]=40.5866109;  waypoints[3][1]=-74.8899582;  
waypoints[4][0]=40.586474;   waypoints[4][1]=-74.8899096;  
waypoints[5][0]=40.5864558;  waypoints[5][1]=-74.8900184;  
waypoints[6][0]=40.586437;   waypoints[6][1]=-74.8901498;  
waypoints[7][0]=40.5864353;  waypoints[7][1]=-74.8902253;  
waypoints[8][0]=40.5864269;  waypoints[8][1]=-74.8902731;  
waypoints[9][0]=40.5864197;  waypoints[9][1]=-74.8903461;  
waypoints[10][0]=40.5864067; waypoints[10][1]=-74.890442;  
waypoints[11][0]=40.5863818; waypoints[11][1]=-74.8907024;  
waypoints[12][0]=40.586349;  waypoints[12][1]=-74.8910185;  
waypoints[13][0]=40.5863131; waypoints[13][1]=-74.8913619;  
waypoints[14][0]=40.586293;  waypoints[14][1]=-74.8915772;  
waypoints[15][0]=40.5862602; waypoints[15][1]=-74.8918713;  
waypoints[16][0]=40.586239;  waypoints[16][1]=-74.8920325;  
waypoints[17][0]=40.5862096; waypoints[17][1]=-74.8922463;  
waypoints[18][0]=40.5862208; waypoints[18][1]=-74.8923703;  
waypoints[19][0]=40.5862304; waypoints[19][1]=-74.8924291;  
waypoints[20][0]=40.586267;  waypoints[20][1]=-74.8925109;  
waypoints[21][0]=40.586328;  waypoints[21][1]=-74.8926005;  
waypoints[22][0]=40.5865234; waypoints[22][1]=-74.8928368;  
waypoints[23][0]=40.5866553; waypoints[23][1]=-74.8930042;  
waypoints[24][0]=40.5867578; waypoints[24][1]=-74.893129;  
waypoints[25][0]=40.586844;  waypoints[25][1]=-74.8932311;  
waypoints[26][0]=40.5869495; waypoints[26][1]=-74.8933179;  
waypoints[27][0]=40.5871612; waypoints[27][1]=-74.8934363;  
waypoints[28][0]=40.5875817; waypoints[28][1]=-74.8936145;  
waypoints[29][0]=40.588201;  waypoints[29][1]=-74.8938565;  
waypoints[30][0]=40.5884069; waypoints[30][1]=-74.8939454;  


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

void loop()
{
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update

    if (feedgps()){
      newdata = true; 
      //Sets the waypoint using the waypoint function and distance stored in localData[4]
      waypoint(localData[7]);
     //Runs TinyGPS only when we have an update. Use the waypoint coordinates for distance and heading calculatons
      gpsdump(gps,WayPT_Lat,WayPT_Lon);
     }
      
/*Check to see if there is new GPS data.  Also check to see if the GPS heading has changed (it tends to lag).
If both are new then send to Kalman filter with the current gyro rate.  The kalman is initialized with
the previous GPS heading. 

Otherwise estimate the current heading uses only the gyro and past GPS heading.
*/

 if (newdata){
     headingKalmanInit(localData[3]);// initialize with old heading
     localData[3] = localData[2]; // The new GPS heading is now the previous one
     localData[2] = headingKalman((millis() - startDt), localData[2], true, (localData[9]), true); // Gyro in deg/ms
     localData[4] = localData[2] + (localData[4] - localData[2]); // corrects gps angle based on new gyro calculated angle
     // Get Gyroscope angular rotation rate to be used for the next kalman update
     startDt = millis(); // timer for kalman updates
     localData[11] = 1; //Bool for new GPS update  
     // Store the angular rate now to be used the next time the Kalman filter runs.
      getGyroRate();
      localData[9] = gyroAngles[1];  
      actAngle =  localData[4];
 }
 
 else{ 
     getGyroRate(); // heading is calculated off only the gyro so it always needs to be updated
     actAngle = localData[4];
     
 }   
// Clamp to 0-360 range
 while (actAngle < 0) actAngle += 360.0;
 while (actAngle >= 360.0) actAngle -= 360.0;

// Store actAngle here for logging   
  localData[5] = actAngle;

// Calculate the difference between waypoint heading (Setpoint) and our current heading 
  angleDiff = localData[6] - actAngle;
 
// Take the shortest turn to the waypoint heading 
  if (angleDiff > 180) {
    angleDiff = (angleDiff - 360);
  }
  
  if (angleDiff < -180) {
    angleDiff = (angleDiff + 360);
  }
  
  localData[8] = angleDiff;
  
  
/*
// Read in values for the PID pots first, Map to 0-10, Then store in array for printing and saving 
  sensorValue1 = map(analogRead(sensorPin1),0,1024,0,10);
  sensorValue2 = map(analogRead(sensorPin2),0,1024,0,10);
  sensorValue3 = map(analogRead(sensorPin3),0,1024,0,10);
  localData[12] = sensorValue1;
  localData[13] = sensorValue2;
  localData[14] = sensorValue3;
*/


// PID and motor drive 
  double gap = abs(0-angleDiff); //distance away from setpoint
   
  if(gap<gapDist)
  {  //we're close to setpoint, use conservative tuning parameters
    steering = updatePid(0, angleDiff, consK, consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
    steering =  updatePid(0, angleDiff, aggK, aggKp, aggKi, aggKd);
  }
   
// Drive steering servo 
  driveSteering(-steering);

  for ( int i = 0; i < 12; i = i + 1) {
          Serial.print(localData[i]); Serial.print(" ");
           } 
           Serial.println();


// Print localData array to SD card ////
  logger();

// Print to LCD ////
  lcdPrint();

//reset gps update value 
  localData[11] = 0; 


  // *********************** loop timing control **************************
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
  
  

}


