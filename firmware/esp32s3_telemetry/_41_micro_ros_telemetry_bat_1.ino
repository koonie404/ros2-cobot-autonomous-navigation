#include <TelemetryRobotNode.h>

#define DOMAIN_ID 36

#define TRACK_WIDTH 0.170f
#define WHEEL_RADIUS 0.0325f // (= 0.065/2) 

NodeStatus          status;
TelemetryRobotNode  robot;

const unsigned long interval = 10000UL;
unsigned long prev_us;

void setup() {
    Serial.begin(115200);

    TelemetryRobotNode_begin(&robot, &status, DOMAIN_ID, TRACK_WIDTH, WHEEL_RADIUS);

    prev_us = micros();
}

void loop() {
    // ROS 통신 처리
    BaseRobotNode_spin(&robot.robot.base);

    unsigned long curr_us = micros();
    if(curr_us - prev_us >= interval) {
        double dt = (curr_us - prev_us)/1000000.0;
        prev_us = curr_us;

        TelemetryRobotNode_update(&robot, dt);
    }
}
