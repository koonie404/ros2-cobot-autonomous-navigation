#include "TelemetryRobotNode.h"

#define BATTERY_PIN 3

static void _telemetry_pub_init(TelemetryRobotNode *self) {
  self->telemetry_msg.data.data = self->telemetry_buf;
  self->telemetry_msg.data.size = MSG_SIZE;
  self->telemetry_msg.data.capacity = MSG_SIZE;

  rclc_publisher_init_best_effort(
    &self->telemetry_pub,
    &self->robot.base.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "esp32s3/telemetry"
  );
}

static void _telemetry_publish(
  TelemetryRobotNode *self,
  int32_t leftPos, 
  int32_t rightPos, 
  int32_t leftVel, 
  int32_t rightVel,
  int32_t gyroZCalib,
  int32_t batVoltage) {        
  self->telemetry_buf[0] = leftPos;
  self->telemetry_buf[1] = rightPos;
  self->telemetry_buf[2] = leftVel; // lRPM = leftVel/100.0
  self->telemetry_buf[3] = rightVel; // rRPM = rightVel/100.0
  self->telemetry_buf[4] = gyroZCalib;
  self->telemetry_buf[5] = batVoltage;

  rcl_publish(&self->telemetry_pub, &self->telemetry_msg, NULL);
}

static int32_t _getBatteryValue() {
  int adc_raw = analogRead(BATTERY_PIN);
  return adc_raw;
}

static double _getBatteryVoltage() {
  int32_t adc_raw = _getBatteryValue();

  // 12bit ADC → 0~3.6V (ADC_11db 기준)
  double v_adc = (double)adc_raw / 4095.0 * 3.6;

  // 저항비 보정 (120k + 20k)
  double v_bat = v_adc * 7.0;

  return v_bat;
}

void TelemetryRobotNode_begin(
  TelemetryRobotNode *self,
  NodeStatus *status,
  int domain_id,
  float track_width, 
  float wheel_radius,
  const BaseRobotNode_vtable *vt) {
  
  DiffDriveRobotNode_begin(
    &self->robot, 
    status, 
    domain_id, 
    track_width, 
    wheel_radius);

  MyMPU6050_init(&self->imu, I2C_SDA, I2C_SCL, MPU6050_ADDR);
  MyMPU6050_begin(&self->imu);

  // 1. telemetry publisher 시작
  _telemetry_pub_init(self); 

  // 2. 텔레메트리 전용 퍼블리셔 초기화
  NodeStatus_publishDebug(self->robot.base.status, "Telemetry System Initialized");
}

void TelemetryRobotNode_update(TelemetryRobotNode *self, double dt) {
  // 1. 모터 제어 주기 처리
  DiffDriveRobotNode_update(&self->robot, dt);
  
  long lPP, rPP;
  double lRPM, rRPM;
  DiffDriveController_getPresentPositions(&self->robot.ddc, &lPP, &rPP);
  DiffDriveController_getPresentRPMs(&self->robot.ddc, &lRPM, &rRPM);

  double gyro_deg_s = MyMPU6050_getGyroZ_DegPerSec(&self->imu);

  double batVoltage = _getBatteryVoltage();

  int32_t leftPos = (int32_t)lPP;
  int32_t rightPos = (int32_t)rPP;
  int32_t leftVel = (int32_t)(lRPM*100);
  int32_t rightVel = (int32_t)(rRPM*100);
  int32_t gyro_deg_s_100 = (int32_t)(gyro_deg_s*100);
  int32_t batVoltage_100 = (int32_t)(batVoltage*100);
  
  _telemetry_publish(self, leftPos, rightPos, leftVel, rightVel, gyro_deg_s_100, batVoltage_100); // telemetry publisher

}

// 1. 업로드
// 2. ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyS0 -b 115200
// 3. ros2 topic list
// 4. ros2 topic echo /esp32s3/telemetry
// 5. ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.2
// 6. I, J, K, L, < 입력해 보기