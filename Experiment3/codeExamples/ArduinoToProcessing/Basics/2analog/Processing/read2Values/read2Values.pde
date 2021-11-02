/*
DIGF 6037 Creation & Computation
   Kate Hartman & Nick Puckett

   Arduino to Processing - sending 1 value from IMU data

 Based on based on this Lab on the ITP Physical Computing site: 
 https://itp.nyu.edu/physcomp/labs/labs-serial-communication/two-way-duplex-serial-communication-using-an-arduino/
 */

import processing.serial.*; // import the Processing serial library
Serial myPort;              // The serial port


int sensor1;
int sensor2;


void setup() {
  size(1023,1023);
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
  
  ellipse(width/2,height/2,sensor1,sensor2);
  
  

 
}


void serialEvent(Serial myPort) {
  // read the serial buffer:
  String myString = myPort.readStringUntil('\n');
  if (myString != null) {
    myString = trim(myString);
    
    String tempData[] = split(myString,',');
    
    //convert from string to the correct data type
    sensor1 = int(tempData[0]);
    sensor2 = int(tempData[1]);
    
    print("sensor1 says : " + sensor1 + "\t");
    println("sensor2 says : " + sensor2);
    
  }
}
