#define DEBUG_MODULE "MD"

#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "semphr.h"
#include "uwb_api.h"

#define DRONE_ID 20
extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
// extern UWB_message uwb_rx_msg;
extern TaskHandle_t uwbTaskHandle;
extern UWB_measurement last_range_msrm;

SemaphoreHandle_t printSemaphore;

uint32_t packets[20];
uint32_t packets_total; 

void mission_drone_task(void* parameters);
void print_task(void* parameters);
void fly_task(void* parameters);

void appMain()
{
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(DRONE_ID);
    xTaskCreate(mission_drone_task, "mission", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
    vTaskDelay(1000);
    xTaskCreate(fly_task, "flier", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

// float pos_x[] = {0.0, 2.0, -2.0, 2.0, 0.0};
// float pos_y[] = {2.0, 2.0,  0.0, 0.0,-2.0};
// float pos_z[] = {0.02, 0.02, 0.02, 0.02, 0.02};

float pos_x[] = {1.4, 1.4, -1.4, -1.4, 2.0};
float pos_y[] = {-1.1, 1.1,  1.1, -1.1, 0.0};
float pos_z[] = {0.05, 0.05, 0.05, 0.05, 0.05};

float dist_log[10];

void registerAnchorDist(float x, float y, float z, float distance, float std_dev) {
    if (distance < 100.0f && distance > 0.1f) {
        distanceMeasurement_t dist;
        dist.distance = distance;
        dist.x = x;
        dist.y = y;
        dist.z = z;
        dist.stdDev = std_dev;
        estimatorEnqueueDistance(&dist);
    }
}

void mission_drone_task(void* parameters) {
    uwb_set_state(RECEIVE);
    while(1)
    {
        if (xSemaphoreTake(msrmReadySemaphore, 200000 / portTICK_PERIOD_MS)) {
            xSemaphoreGive(printSemaphore);
            packets[last_range_msrm.src_id]++;
            packets_total++;
            dist_log[last_range_msrm.src_id] = last_range_msrm.range;
            registerAnchorDist(pos_x[last_range_msrm.src_id], pos_y[last_range_msrm.src_id], pos_z[last_range_msrm.src_id], last_range_msrm.range, 0.5f);
        }   
    }
}


void print_task(void* parameters) {
    TickType_t last_freq_print = xTaskGetTickCount();
    while(1) {
        if (xSemaphoreTake(printSemaphore, 200000 / portTICK_PERIOD_MS)) {
            // DEBUG_PRINT("%.1f %.1f %.1f %.1f\n", pos_x[last_range_msrm.src_id], pos_y[last_range_msrm.src_id], pos_z[last_range_msrm.src_id], last_range_msrm.range);
            if(xTaskGetTickCount() - last_freq_print > 2000) {
                last_freq_print = xTaskGetTickCount();
                for (uint8_t i=0; i<20; i++) {
                    if(packets[i] > 0)
                        DEBUG_PRINT("Anchor %d: %d meas/sec  \n", i, packets[i] / 2);
                    packets[i] = 0;
                }
                DEBUG_PRINT("Total: %d meas/sec  \n", packets_total / 2);
                DEBUG_PRINT("\n");
                packets_total = 0;
            }
        }
    }
}


LOG_GROUP_START(UWB)
LOG_ADD(LOG_FLOAT, d0, &dist_log[0])
LOG_ADD(LOG_FLOAT, d1, &dist_log[1])
LOG_ADD(LOG_FLOAT, d2, &dist_log[2])
LOG_ADD(LOG_FLOAT, d3, &dist_log[3])
LOG_GROUP_STOP(UWB)
