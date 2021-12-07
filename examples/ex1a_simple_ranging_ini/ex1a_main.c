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

void uwb_initiator(void *parameters);

void appMain() {
    uwb_api_init(ANCHOR_ID);
    uwb_set_state(TRANSMIT);

    xTaskCreate(uwb_initiator, "initiator", configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle);

    while(1)
        vTaskDelay(10000);
}


void uwb_initiator(void *parameters) {
    uint32_t pkts_ok_1sec = 0;

    TickType_t xLastWakeTime, t1, t2;
    const TickType_t xPeriod = 4;

    xLastWakeTime = xTaskGetTickCount();
    uint8_t node_pos[7];
    node_pos[0] = 0;

    t1 = xTaskGetTickCount();
    while(1) {
        // DEBUG_PRINT("Time0: %d \n", xTaskGetTickCount());
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        float range;
        uwb_err_code_e e = uwb_do_4way_ranging_with_node(20, node_pos, &range);

        if(e == UWB_SUCCESS) 
            pkts_ok_1sec++;
        // else
        //     DEBUG_PRINT("Error code: %d \n", e);

        // DEBUG_PRINT("Time1: %d \n", xTaskGetTickCount());
        t2 = xTaskGetTickCount();
        if(t2 - t1 > 2000) {
            float delta = (float)(t2 - t1) / 1000.0f;
            DEBUG_PRINT("Successful in 1sec: %.1f  delta: %.2f \n", (float)pkts_ok_1sec / (float)delta, delta);
            pkts_ok_1sec = 0;
            t1 = xTaskGetTickCount();
        }
    }       

}
