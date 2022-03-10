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
extern TaskHandle_t uwbTaskHandle;

SemaphoreHandle_t printSemaphore;

void uwb_initiator(void *parameters);
void print_task(void* parameters);

void appMain() {
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(ANCHOR_ID);
    uwb_responder_off();

    xTaskCreate(uwb_initiator, "initiator", configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    while(1)
        vTaskDelay(10000);
}

uint32_t msrm_ok_cnt = 0;
void uwb_initiator(void *parameters) {
    const TickType_t xPeriod = 3;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uwb_node_coordinates_t node_pos;
    uint32_t range;
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        // uwb_err_code_e e = uwb_do_4way_ranging_with_node(20, node_pos, &range);
        uwb_err_code_e e = uwb_do_3way_ranging_with_node(20, node_pos);

        if(e == UWB_SUCCESS) 
            msrm_ok_cnt++;

        xSemaphoreGive(printSemaphore);
    }       
}


void print_task(void* parameters) {
    TickType_t t = xTaskGetTickCount();
    while(1) {
        if (xSemaphoreTake(printSemaphore, 200000 / portTICK_PERIOD_MS)) {
            if(xTaskGetTickCount() - t > 1000) {
                DEBUG_PRINT("Successful in 1sec: %.1f \n", (float)msrm_ok_cnt * (float)pdMS_TO_TICKS(1000) / (float)(xTaskGetTickCount() - t));
                t = xTaskGetTickCount();
                msrm_ok_cnt = 0;
            }
        }
    }
}
