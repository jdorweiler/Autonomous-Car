// Sensors Modules.  Gyroscope and Accelerometer

float getGyroRate() {       
  
  pastMillis = millis();
  int GYR_Y = getGyroValues();
  if (GYR_Y >= 25000) {
   GYR_Y = GYR_Y - 65536; 
  }
  GYR_Y = -GYR_Y;
  pastMicros = currMicros;
  currMicros = micros();
// drift range for the gyro.  Experimentaly determined
  if(GYR_Y >= 45 || GYR_Y <= -15)
  {
      dt = (float) (currMicros-pastMicros)/10000.0;
      degreesPerSecond = (float)GYR_Y *0.00068 ; // this scale factor was determined mostly by trial and error
      localData[4] += (degreesPerSecond) * dt;
  }
  gyroAngles[1] = degreesPerSecond;
  gyroAngles[0] = gyroAngle; 
  return 0.0;
}

float getAccAngle() {
  float r;
  int regAddress = 0x32;    //first axis-acceleration-data register on the accelerometer
  readFrom(ACC, regAddress, TO_READ, buff); //read the acceleration data from the accelerometer  
   ACC_X = (((int)buff[1]) << 8) | buff[0];
   ACC_Y = (((int)buff[3])<< 8) | buff[2];
   ACC_Z = (((int)buff[5]) << 8) | buff[4];
   //ACC_X = (ACC_X * 0.00402832)*9.8;


   return ACC_X, ACC_Z;
}

//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.send(address);        // send register address
   Wire.send(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.send(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.receive(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//---------------- gyroscope set up ------------------------------------------

int setupgyroscope(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeTo(gyroscope_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeTo(gyroscope_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeTo(gyroscope_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeTo(gyroscope_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeTo(gyroscope_Address, CTRL_REG4, 0b00010000);
  }else{
    writeTo(gyroscope_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeTo(gyroscope_Address, CTRL_REG5, 0b00000000);
}


// ------------------ read gyroscope angles ---------------------

float getGyroValues(){
  float gx, gy, gz;

  byte xMSB = readRegister(gyroscope_Address, 0x29);
  byte xLSB = readRegister(gyroscope_Address, 0x28);
  gx = ((xMSB << 8) | xLSB);
 
  byte yMSB = readRegister(gyroscope_Address, 0x2B);
  byte yLSB = readRegister(gyroscope_Address, 0x2A);
  gy = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(gyroscope_Address, 0x2D);
  byte zLSB = readRegister(gyroscope_Address, 0x2C);
  gz = ((zMSB << 8) | zLSB);
  
  return gz;
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.send(address);       // send register address
    Wire.send(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.send(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.receive();
    return v;
}

