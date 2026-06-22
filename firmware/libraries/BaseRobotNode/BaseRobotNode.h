// =============================================
// BaseRobotNode.h
// =============================================
#pragma once

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "NodeStatus.h"

// ── 전방 선언 ─────────────────────────────────────────────────────────────────
typedef struct BaseRobotNode BaseRobotNode;

// ── vtable ────────────────────────────────────────────────────────────────────
typedef struct {
    void (*processTwist)(BaseRobotNode *self, float linear_x, float angular_z);
} BaseRobotNode_vtable;

// ── 구조체 ────────────────────────────────────────────────────────────────────
struct BaseRobotNode {
    const BaseRobotNode_vtable    *vt;

    // protected
    NodeStatus                    *status;

    // private
    int                            domain_id;
    rcl_allocator_t                allocator;
    rclc_support_t                 support;
    rclc_executor_t                executor;
    rcl_node_t                     node;

    rcl_subscription_t             twist_sub;
    geometry_msgs__msg__Twist      twist_msg;
    rcl_timer_t                    heartbeat_timer;
};

// ── API ───────────────────────────────────────────────────────────────────────
void BaseRobotNode_begin(BaseRobotNode *self,
                        NodeStatus *status,
                        int domain_id,
                        const BaseRobotNode_vtable *vt = nullptr);
void BaseRobotNode_spin(BaseRobotNode *self);
