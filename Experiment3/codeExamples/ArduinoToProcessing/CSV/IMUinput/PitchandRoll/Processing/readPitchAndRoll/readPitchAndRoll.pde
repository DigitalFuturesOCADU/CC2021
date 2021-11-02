/*
DIGF 6037 Creation & Computation
   Kate Hartman & Nick Puckett
   
   Arduino to Processing - sending 2 float values from the IMU

 Based on based on this Lab on the ITP Physical Computing site: 
 https://itp.nyu.edu/physcomp/labs/labs-serial-communication/two-way-duplex-serial-communication-using-an-arduino/
 */

import processing.serial.*; // import the Processing serial library
Serial myPort;              // The serial port

float pitch;
float roll;


void setup() {
  size(500,500);
  // List all the available serial ports in the console
  printArray(Serial.list());

  // Change the 0 to the appropriate number of the serial port
  // that your microcontroller is attached to.
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  // read incoming bytes to a buffer
  // until you get a linefeed (ASCII 10):
  myPort.bufferUntil('\n');
}


void draw() 
{
  background(255);
  fill(255,0,0);
  
  ellipse(width/2,height/2,pitch,roll);
 
}


void serialEvent(Serial myPort) {
  // read the serial buffer:
  String myString = myPort.readStringUntil('\n');
  if (myString != null) {
    // println(myString);
    myString = trim(myString);

    // split the string at the commas
    // and convert the sections into floats:
    float sensors[] = float(split(myString, ','));
    
    pitch = sensors[0];
    roll = sensors[1];
    
    print("Pitch : " + pitch + "\t");
    print("Roll : " + roll + "\t");
    println();
    
  }
}
