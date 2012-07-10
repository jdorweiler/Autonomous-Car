#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "Arduino.h"



float GYR_Y, ACC_X, ACC_Y  , ACC_Z , latDec, lonDec;                                        

#define ACC (0x53)    //accelerometer address
#define TO_READ (6)      //num of bytes to read each time (two bytes for each axis)
#define   LINE_END              10                             // \n
#define   SPLIT                 58                             // :
#define CTRL_REG1 0x20 // gyroscope stuff
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int STD_LOOP_TIME = 9;
int gyroscope_Address = 105; //I2C address of the gyroscope
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;

int ACC_angle;
int GYRO_rate;


byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];          //string buffer to transform data before sending it to the serial port


//#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
//#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

#define GPSRATE 9600

#define BUFFSIZ 90 // plenty big

unsigned long time1, time2, totaltime ;

// global variables
char buffer[BUFFSIZ];        // string buffer for the sentence
char *parseptr;              // a character pointer for parsing
char buffidx;                // an indexer into the buffer

// The time, date, location data, etc.
uint8_t hour, minute, second, year, month, date;
uint32_t latitude, longitude, trackangle;
uint8_t groundspeed;
char latdir, longdir;
char status;

Servo ser_esc;
Servo ser_steer;

void setup() 
{ 
  Serial.begin(57600);
  Serial1.begin(9600);
  Wire.begin();         
//  Serial.println("Turning on the accelerometer");
  writeTo(ACC, 0x2D, 0);      
  writeTo(ACC, 0x2D, 16);
  writeTo(ACC, 0x2D, 8);
 
  setupgyroscope(2000); // Configure to  - 250, 500 or 2000 deg/sec  

  // connect to the GPS at the desired rate
  //mySerial.begin(GPSRATE);
   
  Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  Serial1.println(PMTK_SET_NMEA_UPDATE_5HZ);
  
  ser_esc.attach(8);
  ser_steer.attach(9);
delay(1500); //wait for the sensors to be ready 

} 
 
uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  return d;
}

void readline(void) {
  char c;
  
  buffidx = 0; // start at begninning
  while (1) {
      c=Serial1.read();
      if (c == -1)
        continue;
    //  Serial.print(c);
      if (c == '\n')
        continue;
      if ((buffidx == BUFFSIZ-1) || (c == '\r')) {
        buffer[buffidx] = 0;
        return;
      }
      buffer[buffidx++]= c;
  }
}
 
void loop() 
{ 

  ser_esc.write(1500);
  uint32_t tmp;
  
  //Serial.print("\n\rRead: ");
  readline();
  
  // check if $GPRMC (global positioning fixed data)
  if (strncmp(buffer, "$GPRMC",6) == 0) {
    
    // hhmmss time data
    parseptr = buffer+7;
    tmp = parsedecimal(parseptr); 
    hour = tmp / 10000;
    minute = (tmp / 100) % 100;
    second = tmp % 100;
    
    parseptr = strchr(parseptr, ',') + 1;
    status = parseptr[0];
    parseptr += 2;
    
    // grab latitude & long data
    // latitude
    latitude = parsedecimal(parseptr);
    if (latitude != 0) {
      latitude *= 10000;
      parseptr = strchr(parseptr, '.')+1;
      latitude += parsedecimal(parseptr);
    }
    parseptr = strchr(parseptr, ',') + 1;
    // read latitude N/S data
    if (parseptr[0] != ',') {
      latdir = parseptr[0];
    }
    
    //Serial.println(latdir);
    
    // longitude
    parseptr = strchr(parseptr, ',')+1;
    longitude = parsedecimal(parseptr);
    if (longitude != 0) {
      longitude *= 10000;
      parseptr = strchr(parseptr, '.')+1;
      longitude += parsedecimal(parseptr);
    }
    parseptr = strchr(parseptr, ',')+1;
    // read longitude E/W data
    if (parseptr[0] != ',') {
      longdir = parseptr[0];
    }
    

    // groundspeed
    parseptr = strchr(parseptr, ',')+1;
    groundspeed = parsedecimal(parseptr);

    // track angle
    parseptr = strchr(parseptr, ',')+1;
    trackangle = parsedecimal(parseptr);


    // date
    parseptr = strchr(parseptr, ',')+1;
    tmp = parsedecimal(parseptr); 
    date = tmp / 10000;
    month = (tmp / 100) % 100;
    year = tmp % 100;
    
     Serial.print(latitude); Serial.print(",");
     Serial.print(longitude); Serial.print(",");
     time2 = millis();
     totaltime = (time2-time1);
     Serial.print(totaltime);Serial.print(",");
     getAccAngle();
     //Serial.print(ACC_X); Serial.print(","); Serial.print(ACC_Y); Serial.print(",");
     time1 = time2;
     Serial.print(groundspeed);Serial.print(",");
     
     Serial.println(trackangle); 
     
  }
}

