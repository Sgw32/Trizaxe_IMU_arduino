#include "MPU.h"

uint8_t IMUAddress = 0x68;

int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

uint32_t timer;

void i2cInit(){
  Wire.begin();
  i2cWrite(0x6B,0x00); // Disable sleep mode      
  timer = micros();
}

void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
  return data;
}

void mpuGetRawData( mpuRawData* obj ){
  uint8_t* data = i2cRead(0x3B,14);
  obj->accX = ((data[0] << 8) | data[1]);
  obj->accY = ((data[2] << 8) | data[3]);
  obj->accZ = ((data[4] << 8) | data[5]);
  obj->tempRaw = ((data[6] << 8) | data[7]);
  obj->gyroX = ((data[8] << 8) | data[9]);
  obj->gyroY = ((data[10] << 8) | data[11]);
  obj->gyroZ = ((data[12] << 8) | data[13]);
  return;
}


