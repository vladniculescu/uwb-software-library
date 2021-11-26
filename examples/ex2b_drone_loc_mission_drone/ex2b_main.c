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

void mission_drone_task(void* parameters);

void appMain()
{
    uwb_api_init(DRONE_ID);
    xTaskCreate(mission_drone_task, "mission", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
}

void mission_drone_task(void* parameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uwb_set_state(RECEIVE);
    float anchor_pos[3];
    uint8_t prev_msg_src_id;
    while(1)
    {
        if (xSemaphoreTake(msgReadySemaphore, 200000 / portTICK_PERIOD_MS))
        {
            if(uwb_rx_msg.code == UWB_RANGE_INIT_NO_COORDS_MSG)
            {
                prev_msg_src_id = uwb_rx_msg.src;
                for(uint8_t i=0; i<3; i++)
                    anchor_pos[i] = (float)(uwb_rx_msg.data[2*i] * 256 + uwb_rx_msg.data[2*i+1]);
            }
        }
        if (xSemaphoreTake(msrmReadySemaphore, 200000 / portTICK_PERIOD_MS))
        {
            if(uwb_rx_msg.src == prev_msg_src_id)
            {
                DEBUG_PRINT("ANCHOR %d   X: %f  Y: %f  Z: %f   Range: %f \n", uwb_rx_msg.src, anchor_pos[0], anchor_pos[1], anchor_pos[2], last_range_msrm);
                registerAnchorDist(anchor_pos[0], anchor_pos[1], anchor_pos[2], last_range_msrm, 0.4f);
            }
        }   
    }
}
