#include "quaternionFilters.h"
#include "MPU9250.h"

#define SerialDebug false  // Set to true to get Serial output for debugging
#define PrintRate 100 //Serial print update rate
#define MagDeclination 4.5 //Magnetic declination of San Luis Huexotla, Texcoco
#define myLed 13 //Set up pin 13 lED for toggling

//macros
#define q(A) (*(getQ()+A))

//function prototypes
void printRotation(float*);
void highPassF(float*,float*,float*);

// Global variables
float uVel[3] = {0.0,0.0,0.0}; // current unfiltered linear velocity
float uVelPrev[3] = {0.0,0.0,0.0}; // previous unfiltered linear velocity
float vel[3] = {0.0,0.0,0.0}; // filtered linear velocity

float uPos[3] = {0.0,0.0,0.0}; // current unfiltered Linear position
float uPosPrev[3] = {0.0,0.0,0.0}; // previous unfiltered Linear position
float pos[3] = {0.0,0.0,0.0}; // filtered Linear position

int conv = 500; // wait for algorithm convergence

MPU9250 myIMU;

void setup()
{
    pinMode(myLed,OUTPUT);
    digitalWrite(myLed,HIGH);

    Wire.begin();
    Serial.begin(38400);
 
    //Read WHO_I_AM register, this a good test for communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    if (c != 0x71) //i2c adress of the Acc+gyro and the Magnetometer
    {
        Serial.println("Could not connect to MPU9250");
        Serial.println("Communication failed, abort");
    } //if (c == 0x71)
    else
    {
        //Start by performing a self test witout reporting values
        myIMU.MPU9250SelfTest(myIMU.selfTest);

        // Calibrate gyro and accelerometers, load biases in bias registers
        //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

        //Initialize device for active mode read of Acc,gyro an temperature
        myIMU.initMPU9250();

        //After initialize the Acc+gyro, we can acces to the magnetometer
        byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
        #if SerialDebug
            Serial.print("MPU9250 adress: ");
            Serial.println(c,HEX);
            Serial.print("AK8963 adress: ");
            Serial.println(d,HEX);
        #endif
        
        if (d == 0x48)
        {
            // Get factory magnetometer calibration from AK8963 ROM
            myIMU.initAK8963(myIMU.factoryMagCalibration);
            //Get sensor resolutions, to change the resolution we need to edit the library header file
            myIMU.getAres(); //by default, 2g
            myIMU.getGres(); //by default, 250 dps
            myIMU.getMres(); //by default, 16 bits
            //Calculate magnetometer bias and scale
            //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
            //*****Magnetometer bias and scale user values*****
            myIMU.magBias[0] = 576.475;
            myIMU.magBias[1] = 11.9150;
            myIMU.magBias[2] = -360.2050;
            //**********
        } //if (d == 0x48)
        else
        {
            Serial.println("Failed to connect AK8963");
        }
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
    if (conv != 0 )
    {
        conv--;
    }
    else 
    {
    //Quaternion to rotation matrix
    float R[9] = { 1-( 2*q(2)*q(2) )-( 2*q(3)*q(3) ), 2*q(1)*q(2)-2*q(0)*q(3), 2*q(1)*q(3)+2*q(0)*q(2),
                   2*q(1)*q(2)+2*q(0)*q(3), 1-( 2*q(1)*q(1) )-( 2*q(3)*q(3) ), 2*q(2)*q(3)-2*q(0)*q(1),
                   2*q(1)*q(3)-2*q(0)*q(2), 2*q(2)*q(3)+2*q(0)*q(1), 1-( 2*q(1)*q(1) )-( 2*q(2)*q(2) ) }; 
    // 'tilt-compensated acceleration' and calculate the acceleration in earth frame
    float v[3] = {R[0]*myIMU.ax + R[1]*myIMU.ay + R[2]*myIMU.az, 
                  R[3]*myIMU.ax + R[4]*myIMU.ay + R[5]*myIMU.az,
                  (R[6]*myIMU.ax + R[7]*myIMU.ay + R[8]*myIMU.az) - 1  };

    // Calculate unfiltered linear velocity (integrate linear acceleration)
    uVel[0] = uVelPrev[0] + myIMU.deltat*(v[0]*9.81);
    uVel[1] = uVelPrev[1] + myIMU.deltat*(v[1]*9.81);
    uVel[2] = uVelPrev[2] + myIMU.deltat*(v[2]*9.81);
    // Filter velocity
    highPassF(uVel,uVelPrev,vel);

    //Calculate unfiltered linear position
    uPos[0] = uPosPrev[0] + uVel[0]*myIMU.deltat;
    uPos[1] = uPosPrev[1] + uVel[1]*myIMU.deltat;
    uPos[2] = uPosPrev[2] + uVel[2]*myIMU.deltat;
    // Filter position
    highPassF(uPos,uPosPrev,pos);

    //Serial print at independent rate of data rates
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > PrintRate)
    {
            /*Serial.print(q(0),8);
            Serial.print(" ");
            Serial.print(q(1),8);
            Serial.print(" ");
            Serial.print(q(2),8);
            Serial.print(" ");
            Serial.println(q(3),8);*/


            Serial.print(v[0]);
            Serial.print(" ");
            Serial.print(v[1]);
            Serial.print(" ");
            Serial.println(v[2]);

        myIMU.count = millis();
        myIMU.sumCount = 0;
        myIMU.sum = 0; 
    }//if (myIMU.delt_t > PrintRate)
    }
}//loop

void printRotation(float *R)
{
    Serial.print(R[0]);
    Serial.print(" ");
    Serial.print(R[1]);
    Serial.print(" ");
    Serial.println(R[2]);
    Serial.print(" ");
    Serial.print(R[3]);
    Serial.print(" ");
    Serial.print(R[4]);
    Serial.print(" ");
    Serial.println(R[5]);
    Serial.print(" ");
    Serial.print(R[6]);
    Serial.print(" ");
    Serial.print(R[7]);
    Serial.print(" ");
    Serial.println(R[8]);
    Serial.print(" ");
    Serial.println(" ");
}

void highPassF(float* Xn,float* Xprev,float* Y)
{
    for (int k=0;k<3;k++)
    {
    Y[k] = 0.99*Xn[k] - 0.99*Xprev[k] + 0.99*Y[k];
    Xprev[k] = Xn[k];
    }
}