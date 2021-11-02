#include <six_axis_comp_filter.h>

/* DIGF 6037 Creation & Computation
 * Kate Hartman & Nick Puckett
 * 
 * 
 * This example gets stable Pitch and Roll angles from the internal IMU
 * on the Arduino Nano33 BLE using a complimentary filter
 * 

 * This example sends these values to processing in a comma separated protocol
 */



#include <Arduino_LSM9DS1.h>



CompSixAxis CompFilter(0.1, 2); //define the filter object


float pitch;
float roll;

long lastSend;
int sendRate = 20;

void setup() 
{
  Serial.begin(9600);
  
  //Call .begin() to configure the IMU (Inertial Measurement Unit)
    if (!IMU.begin()) 
    {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}


void loop() 
{
  if(millis()-lastSend>=sendRate)
  {
  calculatePitchAndRoll();
  lastSend=millis();
  }
}


void calculatePitchAndRoll()
{
  float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle;       

  //  Get all motion sensor parameters,
  //  If you're using a different sensor you'll have to replace the values
  if (IMU.accelerationAvailable()) 
  {
    IMU.readAcceleration(accelX, accelY, accelZ);
  }

  //read gyro and store values
  if (IMU.gyroscopeAvailable()) 
  {
  IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
  // Convert these values into angles using the Complementary Filter
  CompFilter.CompAccelUpdate(accelX, accelY, accelZ); // takes arguments in m/s^2
  CompFilter.CompGyroUpdate(gyroX, gyroY, gyroZ); // takes arguments un rad/s 
  CompFilter.CompUpdate();
  CompFilter.CompStart();

  // Get angle relative to X and Y axes and write them to the variables in the arguments
  //in radians
  CompFilter.CompAnglesGet(&xAngle, &yAngle);

  //convert from radians to angles
  pitch = xAngle*RAD_TO_DEG;
  roll = yAngle*RAD_TO_DEG;

  
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(roll);


}
