/*
Watches the serial line for commands sent from the GUI.  This funciton gets called off a timer1 interrupt
 - Stop Car
 - Change PID setting
 - Change Car's speed
 
 First we check for the stop character 'L'
 If there is no stop character then check to see what values need to be updated.  Data is sent in the format 
 S:xxxx Updates speed
 P:xxxx Updates the PID settings
*/

void serialWatch(){
      if (Serial.available()) {
         char stopVal = Serial.read();
         if (stopVal == 'L'){
           start = 'L';
         }
    }
    /*
     if(Serial.available() >= 0) { 
    char inByte = Serial.read();
    if(inByte == 'S') {
      Serial.read(); // discard ':'
      inData = new byte[4];
      Serial.read(inData);
      int intbit = 0;
      intbit = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
      float f  = Float.intBitsToFloat(intbit);
      }
    }
  }
  */
}
