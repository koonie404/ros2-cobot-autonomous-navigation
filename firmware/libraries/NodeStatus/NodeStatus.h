// =============================================
// NodeStatus.h
// =============================================
#pragma once

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

typedef struct {
    rcl_publisher_t       heartbeat_pub;
    rcl_publisher_t       debug_pub;

    std_msgs__msg__String heartbeat_msg;
    std_msgs__msg__String debug_msg;

    char                  heartbeat_buf[50];
    char                  debug_buf[100];

    unsigned long         heartbeat_cnt;
    const char           *node_name;       // "hello from esp32s3" 등
} NodeStatus;

void NodeStatus_begin(NodeStatus *ns, rcl_node_t *node, const char *node_name);
void NodeStatus_publishHeartbeat(NodeStatus *ns);
void NodeStatus_publishDebug(NodeStatus *ns, const char *msg);
