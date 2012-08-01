#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS.h>
#include <PID_v1.h>
#include <SD.h>
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int chipSelect = 53;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

int sensorPin1 = A8;
int sensorPin2 = A9;
int sensorPin3 = A10;

float sensorValue1 = 0;
float sensorValue2 = 0;
float sensorValue3 = 0;


///////////////////////// GPS  ///////////////////////////////////////////////////////////////////
TinyGPS gps;
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define TO_READ (6)      //num of bytes to read each time (two bytes for each axis)
/////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// PID & Motion  ////////////////////////////////////////////////
//Point where it switches from conservative to agressive 
int gapDist=3;
//Aggressive
double aggK=0.8, aggKp=6, aggKi=1, aggKd=1; 
//Conservative
double consK=0.6, consKp=3, consKi=1, consKd=1;


int steering;
Servo ser_esc;
Servo ser_steer;
//////////////////////////////////////////////////////////////////////////////////////////////////



/////////////// Accelerometer and Gyroscope////////////////////////////////////////////////////////////
int ACC_angle;
int GYRO_rate;
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////// Timing for Kalman Filter //////////////////////////////////////////////////////////
int  STD_LOOP_TIME  = 10;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static void gpsdump(TinyGPS &gps, float LONDON_LAT, float LONDON_LON);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);
float dist;
int num; //waypoint number
float WayPT_Lat,WayPT_Lon;
double localData[20]; // Array holding localication data {lat,lon,wayptLat,wayptLon,currAngle,KalmanAngle,Setpoint,dist}
float waypoints[5][5];// Array holding the waypoint coordinates

void setup()
{
  Serial.begin(57600);
  Serial1.begin(9600);
  Wire.begin();
  
// LCD Startup screen  
  lcd.begin(16, 2);
  lcd.print("Autonomous Car ");
  delay(1000);
  lcd.clear();
  if (!SD.begin(chipSelect)) {
    lcd.print("SD card failed");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
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

// Servos and start up steering check      
  ser_esc.attach(8);
  ser_steer.attach(9);
  delay(500); //wait for the sensors to be ready 
  ser_steer.write(1500); delay(100);
  ser_steer.write(1200); delay(100);
  ser_steer.write(1500); delay(100);
  ser_steer.write(1800); delay(100);
  ser_steer.write(1500); ser_esc.write(1500);
   
    pinMode(84, OUTPUT); 
    pinMode(53, OUTPUT); 
    
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
// Set starting coordinates

  
  Serial.println("********************************Started**************************************8");
}

void loop()
{
  /*
  int sensorValue = analogRead(A7);
  float voltage = sensorValue * (5.0 / 1023.0);
  
  while (voltage > 1 ){
    ser_esc.write(1550);
    ser_steer.write(1500);
    delay(1000);
    ser_esc.write(1500);
    delay(1000000);
    break;
  }
  */ 

  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update ??? ////////////////////////////////
  if (millis() - start < 100)
  {
    if (feedgps()){
      newdata = true; 
     
     ///// Runs TinyGPS only when we have an update. Use the waypoint coordinates for distance and heading calculatons///
     gpsdump(gps,WayPT_Lat,WayPT_Lon);
    
     }
  }
waypoint(num,localData[4]/10);

  /// every 0.1sec we update the gyro sensor ////////////////
  if (millis() - start < 50) {
	  GYRO_rate = getGyroRate();
          localData[7] = GYRO_rate;
  

  }
  
 // Initialize Kalman with the current angle if there's a new GPS or the previous angle if there isn't
  headingKalmanInit(localData[2]);
      
// Send current heading from gps and the gyro rate to Kalman filter 
  actAngle = headingKalman(lastLoopTime, localData[2], true, GYRO_rate, true);
  
// Update our current angle and make negative angles 360 or less. 
  if (actAngle < 0) {
    actAngle = 360 + actAngle;
  }
  
  if (actAngle > 360) {
    actAngle = actAngle-360;
  }
  
// Store actAngle here for logging   
  localData[8] = actAngle;
  
// Updates current heading based on Kalman filter for next loop  
  localData[2] = actAngle; 
   
// Calculate the difference between waypoint heading (Setpoint) and our current heading 
  angleDiff = localData[3] - actAngle;
 
// Take the shortest turn to the waypoint heading 
  if (angleDiff > 180) {
    angleDiff = (angleDiff - 360);
  }
  
    if (angleDiff < -180) {
    angleDiff = (angleDiff + 360);
   }
   
localData[9] = angleDiff;

// Read in values for the PID pots first, Map to 0-10, Then store in array for printing and saving 
  sensorValue1 = map(analogRead(sensorPin1),0,1024,0,10);
  sensorValue2 = map(analogRead(sensorPin2),0,1024,0,10);
  sensorValue3 = map(analogRead(sensorPin3),0,1024,0,10);
  localData[12] = sensorValue1;
  localData[13] = sensorValue2;
  localData[14] = sensorValue3;

// *********************** PID and motor drive *****************
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
 
  for ( int i = 0; i < 16; i = i + 1) {
          Serial.print(localData[i]); Serial.print(" ");
           } 
           Serial.println();

// Print localData array to SD card ////
logger();

// Print to LCD ////
lcdPrint();

//reset gps update value 
localData[15] = 0; 


  Serial.print(" "); Serial.print(millis()-start);Serial.println(" ");
  // *********************** loop timing control **************************
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
  
  

}


