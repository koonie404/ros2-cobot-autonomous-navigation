// =============================================
// BaseRobotNode.cpp
// =============================================
#include "BaseRobotNode.h"

// ── 싱글톤 ────────────────────────────────────────────────────────────────────
static BaseRobotNode *_instance = NULL;

// ── static 콜백 ───────────────────────────────────────────────────────────────
static void _twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;
    if (_instance && _instance->vt->processTwist)
        _instance->vt->processTwist(
            _instance,
            (float)msg->linear.x,
            (float)msg->angular.z
        );
}
// 콜백 호출 시점에 필요한 것
// ├── 누구한테 → _instance (전역 포인터, 싱글톤)
// └── 뭘 할지  → vt->processTwist (함수 포인터, vtable)
// 콜백 내용을 일반화하고 싶다
//   → 특정 함수 이름 대신 함수 포인터로 호출
//   → 함수 포인터가 여러 개면 구조체로 묶는 게 자연스럽다
//   → 그게 vtable
//   → _instance가 vtable을 멤버로 가지면 인스턴스와 동작이 함께 묶인다
//   → 그게 C++의 객체

static void _heartbeatTimerCallback(rcl_timer_t *timer, int64_t)
{
    if (timer == NULL) return;
    NodeStatus_publishHeartbeat(_instance->status);
}

// ── processTwist 구현 ─────────────────────────────────────────────────────────
static void _processTwist(BaseRobotNode *base, float v, float w)
{
    // DiffDriveRobotNode *self = (DiffDriveRobotNode *)base; // 필요시 사용

    char msg[64];
    snprintf(msg, sizeof(msg), "v:%.3f(m/s), w:%.3f(rad/s)", v, w);
    NodeStatus_publishDebug(base->status, msg);
}

// ── vtable ────────────────────────────────────────────────────────────────────
static const BaseRobotNode_vtable _vt = {
    .processTwist = _processTwist
};

static void _init(BaseRobotNode *self,
                        NodeStatus *status,
                        int domain_id,
                        const BaseRobotNode_vtable *vt = nullptr)
{
    self->vt        = (vt==nullptr)?&_vt:vt;
    self->status    = status;
    self->domain_id = domain_id;
}

// ── API 구현 ──────────────────────────────────────────────────────────────────
void BaseRobotNode_begin(BaseRobotNode *self,
                        NodeStatus *status,
                        int domain_id,
                        const BaseRobotNode_vtable *vt)
{
    if (_instance != NULL) {
        NodeStatus_publishDebug(self->status, "Error: Another node already exists!");
        return;
    }
    _instance = self;

    _init(self, status, domain_id, vt);

    set_microros_transports();
    delay(2000);

    self->allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, self->allocator);
    rcl_init_options_set_domain_id(&init_options, self->domain_id);
    rclc_support_init_with_options(&self->support, 0, NULL,
                                   &init_options, &self->allocator);

    rclc_node_init_default(&self->node, "esp32s3_node", "", &self->support);

    // subscription
    rclc_subscription_init_default(
        &self->twist_sub,
        &self->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    // heartbeat timer (1000ms)
    rclc_timer_init_default(
        &self->heartbeat_timer,
        &self->support,
        RCL_MS_TO_NS(1000),
        _heartbeatTimerCallback
    );

    // NodeStatus 퍼블리셔 초기화
    NodeStatus_begin(self->status, &self->node, "hello from esp32s3");

    // executor: subscription 1 + timer 1 = 2
    rclc_executor_init(&self->executor, &self->support.context, 2, &self->allocator);
    rclc_executor_add_subscription(
        &self->executor, &self->twist_sub, &self->twist_msg,
        _twistCallback, ON_NEW_DATA
    );
    rclc_executor_add_timer(&self->executor, &self->heartbeat_timer);

    NodeStatus_publishDebug(self->status, "esp32s3 node started");
}

void BaseRobotNode_spin(BaseRobotNode *self)
{
    rclc_executor_spin_some(&self->executor, RCL_MS_TO_NS(0));
}

