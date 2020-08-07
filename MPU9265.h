#ifndef _MPU9265_H_
#define _MPU9265_H_

#include "AbstractSensor.h"

class MPU9265 : public AbstractSensor {
private :
    uint8_t address;


    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void MahonyQuaternionUpdate();
    void getMres();
    void getGres();
    void getAres();
	void readAccelData(int16_t * destination);
	void readGyroData(int16_t * destination);
	void readMagData(int16_t * destination);
    void initMAG(float * destination);
    void initMPU9250();
    void MPU9250SelfTest(float * destination);
    void calibrateMPU9250(float * dest1, float * dest2);
    void writeByte(uint8_t reg, uint8_t data);
    uint8_t readByte(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t count, uint8_t * dest);
    
    const char *accName;
    const char *gyrName;
    const char *magName;  
    const char *yprName;
    const char *distName;
    const char *quatName;

    // Specify sensor full scale
    uint8_t Gscale;
    uint8_t Ascale;
    uint8_t Mscale; // Choose either 14-bit or 16-bit magnetometer resolution
    uint8_t Mmode;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
    float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
    float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
    int16_t tempCount;      // temperature raw count output
    float   temperature;    // Stores the real internal chip temperature in degrees Celsius
    float   SelfTest[6];    // holds results of gyro and accelerometer self test

    // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
    float GyroMeasError;   // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift;   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    // There is a tradeoff in the beta parameter between accuracy and response speed.
    // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
    // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
    // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
    // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
    // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
    // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
    // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
    float beta;   // compute beta
    float zeta;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    uint32_t delt_t; // used to control display output rate
    uint32_t count, sumCount; // used to control display output rate
    float pitch, yaw, roll;
    float deltat, sum;        // integration interval for both filter schemes
    uint32_t lastUpdate, firstUpdate; // used to calculate integration interval
    uint32_t Now;        // used to calculate integration interval

    float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
    float distGyro;		// total distance travelled, computed via gyro sensor data
 
public :
    MPU9265(const char *aName, const char *gName, const char *mName, const char *yName, const char *dName, const char *qName);

    virtual void Init();
    virtual void Sense(OSCBundle *bundle);
};

#endif
