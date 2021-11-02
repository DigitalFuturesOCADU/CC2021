/* DIGF 6037 Creation & Computation
 * Kate Hartman & Nick Puckett
 * 
 * Send single analog value to processing
 * 
 * 
 * 
  */

int sensorPin = A0;
int sensorValue;
  



void setup() 
{
Serial.begin(9600);  

}

void loop() 
{
sensorValue = analogRead(sensorPin);
Serial.println(sensorValue);
delay(100);
}
