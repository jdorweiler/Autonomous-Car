#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS.h>
#include <SD.h>
#include <LiquidCrystal.h>
//#include <plib.h>

// Start Button
const int buttonPin = 83;
int buttonState = 0;
char start = 'L';
// Wheel Encoders
#define pinINT1 6
#define pinINT2 7
#define INT1 1
#define INT2 2
#define ENC1 INT1
#define ENC2 INT2
int wheelEnc1 = 0;
int wheelEnc2 = 0;
int wheelAvg = 0;
int escSpeed = 1500; // car's pwm speed sent to the ESC
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(11, 81, 12, 82, 84, 85);

// CS pin for SD logging
const int chipSelect = 53;
int sdFlag = 0;
int gpsFlag = 0;

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
float gpsAvg[6];
float gpsSum = 0.0;
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PID & Motion  ////////////////////////////////////////////////
//Point where it switches from conservative to agressive 
int gapDist=3;
//Aggressive
double aggK=0.9, aggKp=7, aggKi=4, aggKd=1; 
//Conservative
double consK=0.6, consKp=4, consKi=3, consKd=1;
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


void wheelEncoder1();
void wheelEncoder2();
void setup()
{
  Serial.begin(57600);
  Serial1.begin(9600);
  Wire.begin();
  startup();
  attachCoreTimerService(timer1); 
  pinMode(pinINT2, INPUT);
  pinMode(pinINT1, INPUT);
  attachInterrupt(ENC1, wheelEncoder1, RISING);
  attachInterrupt(ENC2, wheelEncoder2, RISING);  
}
void loop()
{
  while (start == 'H'){  //Main while loop checking to see if the stop character was sent from the laptop
  bool newdata = false;
  unsigned long start = millis();
    if (feedgps()){
      newdata = true; 
      //Sets the waypoint using the waypoint function and distance stored in localData[4]
      waypoint(localData[7]);
     //Runs TinyGPS only when we have an update. Use the waypoint coordinates for distance and heading calculatons
      gpsdump(gps,WayPT_Lat,WayPT_Lon);     
     // GPS FIFO
      //gpsAverage();
    gpsSum = 0;      
     } 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////     
/*Check to see if there is new GPS data.  Also check to see if the GPS heading has changed (it tends to lag).
If both are new then send to Kalman filter with the current gyro rate.  The kalman is initialized with
the previous GPS heading. 
Otherwise estimate the current heading uses only the gyro and past GPS heading.
*/
 if (newdata){
   headingKalmanInit(localData[3]);// initialize with old heading
   localData[3] = localData[2]; // The new GPS heading is now the previous one
   localData[2] = headingKalman((millis() - startDt), localData[2], true, (gyroAngles[1]), true); // Gyro in deg/ms
   startDt = millis();
   actAngle = localData[2]; //Adding the kalman heading to the angle from the gyro the car has turned since 
   gyroAngles[0] = 0;
   wheelAvg = (wheelEnc1+wheelEnc2)/2; // Encoder values at last Kalman update.  Averaged to avoid wheel spin errors.
   
 }
 else{ 
     getGyroRate(); // heading is calculated off only the gyro so it always needs to be updated
    actAngle = localData[2] + gyroAngles[0];
 }  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// Clamp to 0-360 range
 while (actAngle < 0)  actAngle += 360.0;
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
// Print localData array to SD card ////
if (sdFlag) {
  logger();
  lcdPrint();
  serialWatch();
  sdFlag = 0;
  }
//reset gps update value 
  localData[11] = 0; 
  // *********************** loop timing control **************************
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
  }
}
////////////////////////////////////////////////////////
////////  Interrupts   ////////////////////////////////
////////////////////////////////////////////////////////
//core timer interrupt.  This sets the update rate for the sd logging and lcd printing. core_tick_rate = 1ms
uint32_t timer1(uint32_t currentTime) {
  sdFlag = 1;
  gpsFlag = 1;
  return (currentTime + CORE_TICK_RATE*250);
}
// Wheel encoders /////////////////
void wheelEncoder1() {
wheelEnc1++;
}
void wheelEncoder2() {
wheelEnc2++;
}

