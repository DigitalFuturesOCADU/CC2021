/* DIGF 6037 Creation & Computation
 * Kate Hartman & Nick Puckett
 * 
 * Send 2 analog values to processing
 * 
 * 
 * 
  */

int sensorPin1 = A0;
int sensorValue1;
  
int sensorPin2 = A1;
int sensorValue2;


void setup() 
{
Serial.begin(9600);  

}

void loop() 
{
sensorValue1 = analogRead(sensorPin1);
sensorValue2 = analogRead(sensorPin2);
Serial.print(sensorValue1);
Serial.print(",");
Serial.print(sensorValue2);
Serial.println();
delay(100);
}
