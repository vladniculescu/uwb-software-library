#define DEBUG_MODULE "MD"

#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uwb_api.h"

#define DRONE_ID 20
extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
extern TaskHandle_t uwbTaskHandle;
extern float last_range_msrm;

SemaphoreHandle_t printSemaphore;

uint32_t packets[20];
uint32_t packets_total; 

void mission_drone_task(void* parameters);
void print_task(void* parameters);

void appMain()
{
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(DRONE_ID);
    xTaskCreate(mission_drone_task, "mission", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
}

void mission_drone_task(void* parameters) {
    uwb_set_state(RECEIVE);
    while(1)
    {
        if (xSemaphoreTake(msrmReadySemaphore, 200000 / portTICK_PERIOD_MS)) {
            xSemaphoreGive(printSemaphore);
            packets[uwb_rx_msg.src]++;
            packets_total++;
            // registerAnchorDist(anchor_pos[0], anchor_pos[1], anchor_pos[2], last_range_msrm, 0.4f);
        }   
    }
}


void print_task(void* parameters) {
    TickType_t last_freq_print = xTaskGetTickCount();
    while(1) {
        if (xSemaphoreTake(printSemaphore, 200000 / portTICK_PERIOD_MS)) {
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
