#include "MPU.h"
#include "MPU9250.h"

uint8_t IMUAddress = 0x68;

int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

uint32_t timer;

extern MPU9250 IMU;

void i2cInit(){
  
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 9 for a 100 Hz update rate
  IMU.setSrd(9);
  timer = micros();
}

void i2cWrite(uint8_t registerAddress, uint8_t data){

}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  return 0;
}

void mpuGetRawData( mpuRawData* obj ){
  IMU.readSensor();
  obj->accX = IMU.raw_getAccelX_mss();
  obj->accY = IMU.raw_getAccelY_mss();
  obj->accZ = IMU.raw_getAccelZ_mss();
  obj->tempRaw = IMU.raw_getTemperature_C();
  obj->gyroX = IMU.raw_getGyroX_rads();
  obj->gyroY = IMU.raw_getGyroY_rads();
  obj->gyroZ = IMU.raw_getGyroZ_rads();
  return;
}
