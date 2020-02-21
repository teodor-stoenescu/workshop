#include "MPU9265.h"

#include <Wire.h>

#define MPU9250_ADDRESS            0x68
#define MAG_ADDRESS                0x0C

#define GYRO_FULL_SCALE_250_DPS    0x00  
#define GYRO_FULL_SCALE_500_DPS    0x08
#define GYRO_FULL_SCALE_1000_DPS   0x10
#define GYRO_FULL_SCALE_2000_DPS   0x18

#define ACC_FULL_SCALE_2_G        0x00  
#define ACC_FULL_SCALE_4_G        0x08
#define ACC_FULL_SCALE_8_G        0x10
#define ACC_FULL_SCALE_16_G       0x18

#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D

#define INT_PIN_CFG      0x37

#define ACCEL_XOUT_H     0x3B

void MPU9265::WriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(reg);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU9265::ReadByte(uint8_t reg) {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(reg);                   // Put slave register address in Tx buffer
  Wire.endTransmission();             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void MPU9265::ReadBytes(uint8_t reg, uint8_t count, uint8_t *dest) {  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(reg);            // Put slave register address in Tx buffer
  Wire.endTransmission();       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
    
    Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); 
    }
}

MPU9265::MPU9265(const char *aName, const char *gName) {
    accName = aName;
    gyrName = gName;
}

void MPU9265::Init() {
    Wire.begin();
    address = MPU9250_ADDRESS;
        
    // Set accelerometers low pass filter at 5Hz
    WriteByte(ACCEL_CONFIG2, 0x06);
    // Set gyroscope low pass filter at 5Hz
    WriteByte(CONFIG, 0x06);
    
    
    // Configure gyroscope range
    WriteByte(GYRO_CONFIG, GYRO_FULL_SCALE_1000_DPS);
    // Configure accelerometers range
    WriteByte(ACCEL_CONFIG, ACC_FULL_SCALE_4_G);
    // Set by pass mode for the magnetometers
    WriteByte(INT_PIN_CFG, 0x02);
}

void MPU9265::Sense(OSCBundle *bundle) {
    // Read accelerometer and gyroscope
    uint8_t buf[14];

    ReadBytes(ACCEL_XOUT_H, 14, buf);
    
    // Accelerometer
    int16_t ax = -(buf[0] << 8 | buf[1]);
    int16_t ay = -(buf[2] << 8 | buf[3]);
    int16_t az =  (buf[4] << 8 | buf[5]);

    // Gyroscope
    int16_t gx = -(buf[ 8] << 8 | buf[ 9]);
    int16_t gy = -(buf[10] << 8 | buf[11]);
    int16_t gz =  (buf[12] << 8 | buf[13]);

    // Accelerometer
    Serial.print (ax,DEC); 
    Serial.print (":");
    Serial.print (ay,DEC);
    Serial.print (":");
    Serial.print (az,DEC);  
    Serial.print ("   ");
    
    // Gyroscope
    Serial.print (gx,DEC); 
    Serial.print (":");
    Serial.print (gy,DEC);
    Serial.print (":");
    Serial.print (gz,DEC);  
    Serial.print ("\n");
    
    bundle->add(accName).add(ax).add(ay).add(az);
    bundle->add(gyrName).add(gx).add(gy).add(gz);
}