#ifndef _MPU_H_
#define _MPU_H_

#include <Arduino.h>
#include <Wire.h>

struct mpuRawData{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t tempRaw;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};
typedef struct mpuRawData mpuRawData;

void i2cInit();
void i2cWrite(uint8_t registerAddress, uint8_t data);
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes);

void getAngle( double* angleX, double* angleY );
void mpuGetRawData( mpuRawData* obj );

#endif//_MPU6050_H_
