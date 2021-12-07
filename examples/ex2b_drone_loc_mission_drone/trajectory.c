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

#define PI 3.1415926f

float velocity = 0.3f;
float radius = 0.5f;
static setpoint_t setpoint;
uint8_t start = 0;


void flyCircle(point_t pos, float radius, float direction);
void takeoff(float height);
void land(void);
void flyToPoint(point_t endPos);
void headToSetpoint(float x, float y, float z, float yaw);
void positionSet(setpoint_t *setpoint, float x, float y, float z, float yaw);

void fly_task(void* parameters) {
    DEBUG_PRINT("Flytask started!\n");
    vTaskDelay(1000);
    while(1) {
        vTaskDelay(300);
        point_t endPos;
        memset(&endPos, 0, sizeof(endPos));
        endPos.x = 0;
        endPos.y = 0;
        endPos.z = 0.6f;

        while(!start)
            vTaskDelay(1000);
        estimatorKalmanInit();

        takeoff(0.6f);
        flyToPoint(endPos);
        flyCircle(endPos,0.6f,1);
        flyToPoint(endPos);
        land();
    }
}


void flyCircle(point_t pos, float radius, float direction){
    float distance = 2.0f*PI*radius;
    uint16_t steps = distance/velocity*1000/100; //one step is 100ms

    for (int i = 0; i < steps; i++) {
        float a = M_PI + i*2*M_PI/steps;
        float x = (float)direction*(float)cos(a)*radius+direction*radius+pos.x;
        float y = (float)direction*(float)sin(a)*radius+pos.y;
        headToSetpoint(x, y, pos.z, 0);
        vTaskDelay(100);
    }
}

void takeoff(float height)
{
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    int endheight = (int)(100*(height-0.4f));
    for(int i=0; i<endheight; i++)
    {
        headToSetpoint(pos.x, pos.y, 0.4f + (float)i / 100.0f, 0);
        headToSetpoint(0.0f, 0.0f, 0.4f + (float)i / 100.0f, 0);

        vTaskDelay(30);
    }

}

void land(void){

    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    float height = pos.z;
    for(int i=(int)100*height; i>18; i--)
    {
        headToSetpoint(pos.x, pos.y, (float)i / 100.0f, 0);
        vTaskDelay(100);
    }

    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    vTaskDelay(200);

}

void flyToPoint(point_t endPos){
    point_t startPos;
    memset(&startPos, 0, sizeof(startPos));
    estimatorKalmanGetEstimatedPos(&startPos);

    float distX = endPos.x-startPos.x;
    float distY = endPos.y-startPos.y;
    float distance = sqrt(pow(distX,2)+pow(distY,2));

    uint16_t steps = distance/velocity*1000/100; //one step is 100ms

    for (int i = 0; i < steps; i++) {
        headToSetpoint(startPos.x+i*distX/steps, startPos.y+i*distY/steps, endPos.z, 0);
        vTaskDelay(100);
    }
    for (int i = 0; i < 20; i++) {
        headToSetpoint(endPos.x, endPos.y, endPos.z, 0);
        vTaskDelay(100);
    }
}

void headToSetpoint(float x, float y, float z, float yaw)
{
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
