#include "quaternionFilters.h"
#include "MPU9250.h"

#define SerialDebug true  // Set to true to get Serial output for debugging
#define PrintRate 300 //Serial print update rate
#define MagDeclination 4.5 //Magnetic declination of San Luis Huexotla, Texcoco
#define myLed 13 //Set up pin 13 lED for toggling

MPU9250 myIMU;

void setup()
{
    pinMode(myLed,OUTPUT);
    digitalWrite(myLed,HIGH);

    Wire.begin();
    Serial.begin(38400);
 
    //Read WHO_I_AM register, this a good test for communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if ((c == 0x71) && (d == 0x48)) //i2c adress of the Acc+gyro and the Magnetometer
    {
        //Start by performing a self test witout reporting values
        myIMU.MPU9250SelfTest(myIMU.selfTest);

        // Calibrate gyro and accelerometers, load biases in bias registers
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

        //Initialize device for active mode read of Acc,gyro an temperature
        myIMU.initMPU9250();

        // Get factory magnetometer calibration from AK8963 ROM
        myIMU.initAK8963(myIMU.factoryMagCalibration);

        //Get sensor resolutions, to change the resolution we need to edit the library header file
        myIMU.getAres(); //by default, 2g
        myIMU.getGres(); //by default, 250 dps
        myIMU.getMres(); //by default, 16 bits

        //Calculate magnetometer bias and scale
        myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);

        //*****Magnetometer bias and scale user values*****
        myIMU.magBias[0] = 576.475;
        myIMU.magBias[1] = 11.9150;
        myIMU.magBias[2] = -360.2050;
        //********** 
    }//if (c == 0x71 && d == 0x48)
    else
    {
        Serial.println("Could not connect to MPU9250");
        Serial.println("Communication failed, abort");
        abort();
    }
    digitalWrite(myLed,LOW);
}//setup

void loop()
{
    //On interrup, Check if data ready interrupt
    //Reads sensors data
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
        myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
        // Now we'll calculate the accleration value into actual g's
        // This depends on scale being set
        myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
        myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
        
        myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
        
        myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        // Get actual magnetometer value, this depends on scale being set
        myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes* myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
        myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes* myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
        myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes* myIMU.factoryMagCalibration[2] - myIMU.magBias[2];    
    }//(myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    myIMU.updateTime();
    
    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                           myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                           myIMU.mx, -myIMU.mz, myIMU.deltat);

    //Serial print at independent rate of data rates
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > PrintRate)
    {
            Serial.print(*getQ(),8);
            Serial.print(" ");
            Serial.print(*(getQ() + 1),8);
            Serial.print(" ");
            Serial.print(*(getQ() + 2),8);
            Serial.print(" ");
            Serial.println(*(getQ() + 3),8);

        myIMU.count = millis();
        myIMU.sumCount = 0;
        myIMU.sum = 0; 
    }//if (myIMU.delt_t > PrintRate)
}//loop