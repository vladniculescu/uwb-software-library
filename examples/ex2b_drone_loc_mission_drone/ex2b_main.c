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

#define DRONE_ID 20
extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
extern TaskHandle_t uwbTaskHandle;
extern float last_range_msrm;

SemaphoreHandle_t printSemaphore;


void mission_drone_task(void* parameters);
void print_task(void* parameters);

void appMain()
{
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(DRONE_ID);
    xTaskCreate(mission_drone_task, "mission", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
}

void mission_drone_task(void* parameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uwb_set_state(RECEIVE);
    float anchor_pos[3];
    uint8_t prev_msg_src_id;
    while(1)
    {
        // if (xSemaphoreTake(msgReadySemaphore, 200000 / portTICK_PERIOD_MS))
        // {
        //     DEBUG_PRINT("A: code %d   \n", uwb_rx_msg.code);
        //     if(uwb_rx_msg.code == UWB_RANGE_INIT_NO_COORDS_MSG)
        //     {
        //         prev_msg_src_id = uwb_rx_msg.src;
        //         for(uint8_t i=0; i<3; i++)
        //             anchor_pos[i] = (float)(uwb_rx_msg.data[2*i] * 256 + uwb_rx_msg.data[2*i+1]);
        //     }
        // }
        if (xSemaphoreTake(msrmReadySemaphore, 200000 / portTICK_PERIOD_MS))
        {
            xSemaphoreGive(printSemaphore);
            // DEBUG_PRINT("ANCHOR %d  Range: %f \n", uwb_rx_msg.src, last_range_msrm);
            // if(uwb_rx_msg.src == prev_msg_src_id)
            // {
            //     registerAnchorDist(anchor_pos[0], anchor_pos[1], anchor_pos[2], last_range_msrm, 0.4f);
            // }
        }   
    }
}


void print_task(void* parameters) {
    while(1) {
        if (xSemaphoreTake(printSemaphore, 200000 / portTICK_PERIOD_MS)) {
            DEBUG_PRINT("t: %d  ANCHOR %d  Range: %f \n", xTaskGetTickCount(), uwb_rx_msg.src, last_range_msrm);
        }
    }
}
