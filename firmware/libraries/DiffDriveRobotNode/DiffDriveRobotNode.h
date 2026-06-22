// =============================================
// DiffDriveRobotNode.h
// =============================================
#pragma once

#include <BaseRobotNode.h>
#include <DiffDriveController.h>

typedef struct {
    BaseRobotNode  base;    // 반드시 첫 번째 멤버
    float _track_width;
    float _wheel_radius;
    DiffDriveController ddc;
} DiffDriveRobotNode;

void DiffDriveRobotNode_begin(DiffDriveRobotNode *self,
                             NodeStatus *status,
                             int domain_id,
                             float track_width, 
                             float wheel_radius,
                             const BaseRobotNode_vtable *vt = nullptr);
void DiffDriveRobotNode_update(DiffDriveRobotNode *self, double dt);
