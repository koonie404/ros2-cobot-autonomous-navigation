#pragma once

#include <Wire.h>

#define I2C_SDA 40   // SDA로 사용할 GPIO 번호
#define I2C_SCL 39   // SCL로 사용할 GPIO 번호

#define MPU6050_ADDR 0x68
#define GYRO_Z_REG   0x47
#define GYROXYZ_TO_DPS 131.0  // LSB/°/s

typedef struct {
  int _sda;
  int _scl;
  uint8_t _addr;
  int16_t _gyroZOffset;
}  MyMPU6050;

void MyMPU6050_init(MyMPU6050 * self, int sda, int scl, int8_t addr);
void MyMPU6050_begin(MyMPU6050 * self);
int16_t MyMPU6050_getGyroZCalib(MyMPU6050 * self);
double MyMPU6050_getGyroZ_DegPerSec(MyMPU6050 * self);
