#define DEBUG_MODULE "uwb_initiator"

#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uwb_api.h"

#define ANCHOR_ID 0
extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
extern TaskHandle_t uwbTaskHandle;

SemaphoreHandle_t printSemaphore;

void uwb_initiator(void *parameters);
void print_task(void* parameters);

void appMain() {
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(ANCHOR_ID);
    uwb_set_state(TRANSMIT);

    xTaskCreate(uwb_initiator, "initiator", configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    while(1)
        vTaskDelay(10000);
}

uint32_t pkts_ok_1sec = 0;
void uwb_initiator(void *parameters) {
    const TickType_t xPeriod = 4;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uwb_node_coordinates_t node_pos;
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        uint32_t range;
        uwb_err_code_e e = uwb_do_4way_ranging_with_node(20, node_pos, &range);

        if(e == UWB_SUCCESS) 
            pkts_ok_1sec++;

        xSemaphoreGive(printSemaphore);
    }       

}


void print_task(void* parameters) {
    TickType_t t1, t2;
    t1 = xTaskGetTickCount();
    while(1) {
        if (xSemaphoreTake(printSemaphore, 200000 / portTICK_PERIOD_MS)) {
            t2 = xTaskGetTickCount();
            if(t2 - t1 > 2000) {
                float delta = (float)(t2 - t1) / 1000.0f;
                // DEBUG_PRINT("Successful in 1sec: %.1f  delta: %.2f \n", (float)pkts_ok_1sec / (float)delta, delta);
                pkts_ok_1sec = 0;
                t1 = xTaskGetTickCount();
        }
        }
    }
}
