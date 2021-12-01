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
    uint32_t packet_cnt = 0;
    uint32_t pkts_ok_100 = 0;
    uint32_t pkts_ok_1sec = 0;

    TickType_t xLastWakeTime, t1, t2;
    const TickType_t xPeriod = 4;

    xLastWakeTime = xTaskGetTickCount();
    uint8_t node_pos[7];
    node_pos[0] = 0;

    t1 = xTaskGetTickCount();
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        float range;
        uwb_err_code_e e = uwb_do_4way_ranging_with_node(20, node_pos, &range);
        uwb_check_for_errors(e);
        DEBUG_PRINT("Error: %d \n", e);

        if(range > 0.01f) {
            pkts_ok_100++;
            pkts_ok_1sec++;
        }

        if(packet_cnt == 99) {
            // DEBUG_PRINT("Successful out of 100: %d \n", pkts_ok_100);
            pkts_ok_100 = 0;
            packet_cnt = 0;
        }
        else
            packet_cnt++;

        t2 = xTaskGetTickCount();
        if(t2 - t1 > 1000)
        {
            // DEBUG_PRINT("Successful in 1sec: %d \n", pkts_ok_1sec);
            pkts_ok_1sec = 0;
            t1 = xTaskGetTickCount();
        }
    }       

}
