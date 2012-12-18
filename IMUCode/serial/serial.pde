/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
char val;
int x = 0;
void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(11, OUTPUT); 
  Serial.begin(57600);
  
}

void loop() {
  unsigned int startTag = 0xDEAD;
  int a = analogRead(0);
  int b =  200;
  int c = 400;
 
Serial.print(0xff, BYTE); // Sync byte
/*
Serial.print(a);Serial.print(",");Serial.println(b);
*/
Serial.print((a >> 8) & 0xff, BYTE);
Serial.print(a & 0xff, BYTE);
Serial.print((b >> 8) & 0xff, BYTE);
Serial.print(b & 0xff, BYTE);
Serial.print((c >> 8) & 0xff, BYTE);
Serial.print(c & 0xff, BYTE);

  }

