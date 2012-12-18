
#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 Servo myservo2;  // create servo object to control a servo
int potpin = 4;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  Serial.begin(9600);
myservo.attach(29);  // attaches the servo on pin 9 to the servo object 
myservo2.attach(31);
myservo.write(1500);
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(val); 
myservo2.write(val); 
Serial.println(val);
// sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
}
