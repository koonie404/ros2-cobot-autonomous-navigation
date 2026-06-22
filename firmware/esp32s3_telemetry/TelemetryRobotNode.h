#pragma once

#include <DiffDriveRobotNode.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <myMPU6050.h>

#define MSG_SIZE 6

typedef struct {
  DiffDriveRobotNode  robot;
  rcl_publisher_t telemetry_pub;
  std_msgs__msg__Int32MultiArray telemetry_msg;
  int32_t telemetry_buf[MSG_SIZE];
  MyMPU6050 imu;
} TelemetryRobotNode;

void TelemetryRobotNode_begin(TelemetryRobotNode *self,
                             NodeStatus *status,
                             int domain_id,
                             float track_width, 
                             float wheel_radius,
                             const BaseRobotNode_vtable *vt = nullptr);
void TelemetryRobotNode_update(TelemetryRobotNode *self, double dt);
