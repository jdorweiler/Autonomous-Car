
import controlP5.*;
import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
import org.gwoptics.graphics.graph2D.traces.RollingLine2DTrace;
import processing.serial.*;

String version = "Autonomous Car GUI v1.0";
float xx = 0.0;
float yy = 0.0;
float zz = 0.0;
int val;
int ii = 1;
float [] ff = new float[4];
Serial myPort;
int inString;  // Input string from serial port
int lf = 10;      // ASCII linefeed

class eq implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    return xx;
  }    
}

class eq2 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    return yy;
  }    
}

class eq3 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    return zz;
  }    
}

RollingLine2DTrace r,r2,r3;
Graph2D g;
Graph2D f;

ControlP5 cp5;

@ControlElement (properties = { "min=2000", "max=1000", "type=slider", "height=15", "width=80"}, x=10, y=220, label="Speed")
    public int speed = 1500;
@ControlElement (properties = { "min=0", "max=5", "type=slider", "height=15", "width=80"}, x=10, y=320, label="P")
    public float p = 1.00;
@ControlElement (properties = { "min=0", "max=5", "type=slider", "height=15", "width=80"}, x=10, y=350, label="I")
    public float i = 1.00;
@ControlElement (properties = { "min=0", "max=5", "type=slider", "height=15", "width=80"}, x=10, y=380, label="D")
    public float d = 1.00;
@ControlElement (properties = { "min=0", "max=255", "type=slider", "height=15", "width=80"} , x=10, y=550, label="Change Background")
    public float mm = 400;

int startVal = 0;

void setup() {
  size(800, 600);
  
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 57600);
  myPort.bufferUntil(lf);
  
  cp5 = new ControlP5(this);
  
     cp5.addButton("Start")
     .setPosition(100,10)
     .setSize(50,19)
     ;
     
     cp5.addButton("Stop")
     .setPosition(170,10)
     .setSize(50,19)
     ;
     cp5.addButton("Send_Speed")
     .setPosition(150,220)
     .setSize(55,19)
     ;
     cp5.addButton("Send_PID")
     .setPosition(150,300)
     .setSize(55,19)
     ;
     


  r  = new RollingLine2DTrace(new eq() ,100,0.1f);
  r.setTraceColour(0, 0, 0);
  
  r2 = new RollingLine2DTrace(new eq2(),100,0.1f);
  r2.setTraceColour(255, 0, 0);
  
  r3 = new RollingLine2DTrace(new eq3(),100,0.1f);
  r3.setTraceColour(0, 0, 255);
   
  g = new Graph2D(this, 400, 200, false);
  g.setYAxisMax(500);
  g.setYAxisMin(0);
  g.addTrace(r);
  g.addTrace(r2);
  g.position.y = 50;
  g.position.x = 350;
  g.setYAxisTickSpacing(100);
  g.setXAxisMax(5f);
  g.setYAxisLabel("Velocity m/s");
  g.setXAxisLabel("Time");
  
  
  f = new Graph2D(this, 400, 200, false);
  f.setYAxisMax(200);
  f.setYAxisMin(-200);
  f.addTrace(r3);
  f.position.y = 325;
  f.position.x = 350;
  f.setYAxisTickSpacing(25);
  f.setXAxisMax(5f);
  f.setYAxisLabel("Steering Angle");
  f.setXAxisLabel("Time");   
  //noStroke();
  cp5 = new ControlP5(this);
  // Annotations:
  // addControllersFor(PApplet) checks the main sketch for 
  // annotations and adds controllers accordingly.
 cp5.addControllersFor(this);
}

void draw() {
   if(myPort.available() >= 0) {
    char inByte = myPort.readChar();
    if(inByte == 'f') {
      // we expect data with this format f:XXXX
      myPort.readChar(); // discard ':'
      byte [] inData = new byte[4];
      myPort.readBytes(inData);
      int intbit = 0;
      intbit = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
      float f  = Float.intBitsToFloat(intbit);
      ff[ii] = f;
      ii = ++ii;
      xx = ff[2]*3;
      zz = ff[2];
      yy = ff[2]/2;
      if(ii > 3){
        ii = 1;
      }
    }
  }


/* while (myPort.available() >= 9 ) {
    if (myPort.read() == 0xff){
      xx = (myPort.read() << 8) | (myPort.read());
      yy = (myPort.read() << 8) | (myPort.read());
      zz = (myPort.read() << 8) | (myPort.read());
}
 
 } */
  background(145);
 

  g.draw();
  f.draw();
  

  
  textSize(18);
  text("PID Gains", 10, 310); 
  fill(0, 102, 153);
  
  textSize(16);
  text("Velocity", 10, 90); 
  fill(0, 102, 153);
  
  textSize(16);
  text(yy, 170, 90); 
  fill(0, 102, 153);
  
  textSize(16);
  text("Acceleration", 10, 120); 
  fill(0, 102, 153);
  
  textSize(16);
  text(xx, 170, 120); 
  fill(0, 102, 153);
  
  textSize(16);
  text("Dist from waypoint", 10, 150); 
  fill(0, 102, 153);
  
  textSize(16);
  text(yy, 170, 150); 
  fill(0, 102, 153);
  
  textSize(16);
  text("Steering Error (deg.)", 10, 450); 
  fill(0, 102, 153);
  
  textSize(16);
  text(zz, 170, 450); 
  fill(0, 102, 153);
  
  textSize(14);
  text(version, 500, 20); 
  fill(0, 102, 153);

}


void Start() {
          myPort.write('H');
  println("Car Started ");
}
void Send_PID() {
  if (startVal == 0){
      startVal = 1;
    }
  print("Sent PID values: "); print(p); print(",");print(i); print(",");println(d); 
}

void Stop() {
          myPort.write('L');
  println("Car Stopped ");
}

void Send_Speed() {
          myPort.write('L');
  print("Send Speed: "); println(speed);
}


