#include "myMPU6050.h"
#include <Preferences.h>

static Preferences prefs;

void MyMPU6050_init(
  MyMPU6050 * self, 
  int sda, 
  int scl, 
  int8_t addr) {
  self->_sda = sda;
  self->_scl = scl;
  self->_addr = addr;
}

void MyMPU6050_begin(MyMPU6050 * self) {
  Wire.begin(self->_sda, self->_scl);
  Wire.setClock(400000);

  // MPU6050 활성화
  Wire.beginTransmission(self->_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // NVS에서 보정값 읽기 (읽기 전용 모드)
  prefs.begin("imu", true);
  self->_gyroZOffset = prefs.getInt("GyZOff", 0); // 오프셋 불러오기 (없으면 0.0)
  prefs.end();

  Serial.print("Loaded GyZOff = ");
  Serial.println(self->_gyroZOffset);
}

static int16_t MyMPU6050_readGyroZRaw(MyMPU6050 * self) {
  Wire.beginTransmission(self->_addr);
  Wire.write(GYRO_Z_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(self->_addr, 2, true);
  int16_t high = Wire.read();
  int16_t low = Wire.read();
  return (high << 8) | low;
}

// 보정된 각속도 (deg/s)
int16_t MyMPU6050_getGyroZCalib(MyMPU6050 * self) {
  int16_t GyZ = MyMPU6050_readGyroZRaw(self);
  int16_t GyZCalib = GyZ - self->_gyroZOffset;
  return GyZCalib;
}

double MyMPU6050_getGyroZ_DegPerSec(MyMPU6050 * self) {
  int16_t GyZCalib = MyMPU6050_getGyroZCalib(self);
  return GyZCalib/GYROXYZ_TO_DPS;
}