#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motors.h"
#include "estimator_kalman.h"
#include "commander.h"
#include "param.h"
#include "config_params.h"

void takeoff(float height);
void land(void);
void flyToPoint(point_t endPos);
void headToSetpoint(float x, float y, float z, float yaw);
void positionSet(setpoint_t *setpoint, float x, float y, float z, float yaw);
void flyCircle(point_t center, float radius, float phase, uint32_t duration_ms);

uint8_t start = 0;

void fly_task(void* parameters) {
    DEBUG_PRINT("Fly task started!\n");
    vTaskDelay(1000);
    while(1) {

        while(!start)
            vTaskDelay(1000);

        // Reset estimator
        estimatorKalmanInit();

        // Define trajectory circle parameters
        point_t circle_center;
        memset(&circle_center, 0, sizeof(circle_center));
        float radius = 1.0f;
        circle_center.x = 0.0f;
        circle_center.y = 0.0f;
        circle_center.z = 0.8f;

        // Take off 
        takeoff(circle_center.z);

        // Fly circle
        float phase = (float)(DRONE_ID - 20) * 120.0f;
        flyCircle(circle_center, radius, phase, 10000);
        flyCircle(circle_center, radius, phase, 10000);
        land();
        break;
    }
}

void flyCircle(point_t center, float radius, float phase, uint32_t duration_ms){
    uint32_t step_delay_ms = 40;
    uint32_t steps = duration_ms / step_delay_ms;
    float loop_progress;
    phase = phase * (float)M_PI / 180.0f;

    // Go to start
    point_t start_point;
    start_point.x = (float)cos(phase) * radius + center.x;
    start_point.y = (float)sin(phase) * radius + center.y;
    start_point.z = center.z;
    // flyToPoint(start_point);

    for (uint32_t i = 0; i < 40; i++) {
        headToSetpoint(start_point.x, start_point.y, start_point.z, 0);
        vTaskDelay(50);
    }

    // Loop circle
    for (uint32_t i = 0; i < steps; i++) {
        loop_progress = (float)i / (float)steps;
        float angle = loop_progress * 2 * (float)M_PI + phase;
        float x = (float)cos(angle) * radius + center.x;
        float y = (float)sin(angle) * radius + center.y;
        headToSetpoint(x, y, center.z, 0);
        DEBUG_PRINT("%.2f, %.2f \n", x, y);
        vTaskDelay(step_delay_ms);
    }
}


void takeoff(float height) {
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    uint32_t endheight = (uint32_t)(100 * (height - 0.3f));
    for(uint32_t i=0; i<endheight; i++) {
        headToSetpoint(pos.x, pos.y, 0.3f + (float)i / 100.0f, 0);
        vTaskDelay(30);
    }
}

void land(void) {
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    float height = pos.z;
    for(int i=(int)100*height; i>18; i--) {
        headToSetpoint(pos.x, pos.y, (float)i / 100.0f, 0);
        vTaskDelay(100);
    }

    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    vTaskDelay(200);
}

void flyToPoint(point_t targetPos){
    // float avg_vel = 0.5f;
    // uint32_t step_delay_ms = 40;

    // point_t startPos;
    // memset(&startPos, 0, sizeof(startPos));
    // estimatorKalmanGetEstimatedPos(&startPos);

    // float distX = targetPos.x - startPos.x;
    // float distY = targetPos.y - startPos.y;
    // float distance = sqrtf(powf(distX, 2) + powf(distY, 2));

    // float avg_time = 1000.0f * distance / avg_vel;

    // uint32_t steps = (uint32_t)(avg_time / (float) step_delay_ms);
    // for (uint32_t i = 0; i < steps; i++) {
    //     headToSetpoint(startPos.x + i * distX / steps, startPos.y + i * distY / steps, targetPos.z, 0);
    //     vTaskDelay(step_delay_ms);
    // }
    // for (uint32_t i = 0; i < 20; i++) {
    //     headToSetpoint(targetPos.x, targetPos.y, targetPos.z, 0);
    //     vTaskDelay(step_delay_ms);
    // }
}

void headToSetpoint(float x, float y, float z, float yaw)
{
    setpoint_t setpoint;
    positionSet(&setpoint, x, y, z, yaw);
    commanderSetSetpoint(&setpoint, 3);
}

void positionSet(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
    memset(setpoint, 0, sizeof(setpoint_t));

    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;

    setpoint->position.x = x;
    setpoint->position.y = y;
    setpoint->position.z = z;

    setpoint->mode.yaw = modeAbs;

    setpoint->attitude.yaw = yaw;

    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.quat = modeDisable;
}


PARAM_GROUP_START(UWB_COMMANDS)
PARAM_ADD(PARAM_UINT8, start_flying, &start)
PARAM_GROUP_STOP(UWB_COMMANDS)
