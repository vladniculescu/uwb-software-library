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
#include "deca_device_api.h"


#define ANCHOR_ID 0
#define NR_OF_ANCHORS 2
#define RANGING_TIME 5

extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
extern TaskHandle_t uwbTaskHandle;

void anchor_task(void* parameters);

void appMain() {
    uwb_api_init(ANCHOR_ID);
    xTaskCreate(anchor_task, "anchor", configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle);

    while(1)
        vTaskDelay(10000);
}

void anchor_task(void* parameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t anchor_pos[6];
    uwb_set_state(TRANSMIT);
    while(1)
    {
        if(ANCHOR_ID == 0)
        {
            float range;
            vTaskDelayUntil(&xLastWakeTime, NR_OF_ANCHORS*RANGING_TIME);
            uwb_err_code_e e = uwb_do_4way_ranging_with_node(20, anchor_pos, &range);
            // DEBUG_PRINT("Error: %d   \n", e);
        }
        else
        {
            uwb_set_state(RECEIVE);
            if (xSemaphoreTake(msgReadySemaphore, 200000))
            {
                if((uwb_rx_msg.src == 0) && (uwb_rx_msg.code == UWB_RANGE_INIT_NO_COORDS_MSG))
                {
                    uwb_set_state(TRANSMIT);
                    vTaskDelay(ANCHOR_ID*RANGING_TIME);
                    // uwb_do_3way_ranging_with_node(20, anchor_pos);
                    float range;
                    uwb_err_code_e e = uwb_do_4way_ranging_with_node(20, anchor_pos, &range);
                    // DEBUG_PRINT("Error: %d   \n", e);
                }
            }
        }
    }
}
