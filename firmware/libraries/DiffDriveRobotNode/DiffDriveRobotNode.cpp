// =============================================
// DiffDriveRobotNode.cpp
// =============================================
#include "DiffDriveRobotNode.h"
#include <stdio.h>
#include <math.h>

// ── processTwist 구현 ─────────────────────────────────────────────────────────
static void _processTwist(BaseRobotNode* base, float v, float w)
{
	DiffDriveRobotNode* self = (DiffDriveRobotNode*)base;

	float _track_width = self->_track_width;
	float _wheel_radius = self->_wheel_radius;

	// 역기구학 계산
	float left_speed = v - (w * _track_width / 2.0f);
	float right_speed = v + (w * _track_width / 2.0f);

	float left_rpm = left_speed / (2.0f * M_PI * _wheel_radius) * 60.0f + 0.5f;
	float right_rpm = right_speed / (2.0f * M_PI * _wheel_radius) * 60.0f + 0.5f;

	// 실제 모터 장착 방향 보정
	left_rpm = -left_rpm;
	right_rpm = -right_rpm;

	if (fabs(left_rpm) < 1.0f) {
		left_rpm = 0.0f;
	}

	if (fabs(right_rpm) < 1.0f) {
		right_rpm = 0.0f;
	}

	char debug_text[64];
	snprintf(debug_text, sizeof(debug_text), "left_rpm:%.1f, right_rpm:%.1f", left_rpm, right_rpm);
	NodeStatus_publishDebug(base->status, debug_text);

	DiffDriveController_setTargetRPMs(&self->ddc, (double)left_rpm, (double)right_rpm);
}

// ── vtable ────────────────────────────────────────────────────────────────────
static const BaseRobotNode_vtable _vt = {
	.processTwist = _processTwist
};

// ── API 구현 ──────────────────────────────────────────────────────────────────
void DiffDriveRobotNode_begin(DiffDriveRobotNode* self,
	NodeStatus* status,
	int domain_id,
	float track_width,
	float wheel_radius,
	const BaseRobotNode_vtable* vt)
{
	BaseRobotNode_begin(&self->base, status, domain_id, (vt == nullptr) ? &_vt : vt);
	self->_track_width = track_width;
	self->_wheel_radius = wheel_radius;

	DiffDriveController_begin(&self->ddc);
}

void DiffDriveRobotNode_update(DiffDriveRobotNode* self, double dt)
{
	// loop()의 제어 주기에서 호출
	// dt는 호출부에서 계산해서 넘기는 구조로 확장 가능
	DiffDriveController_updateMotorSpeeds(&self->ddc, dt);
}