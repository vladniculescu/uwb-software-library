#define DEBUG_MODULE "AD"

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


#define NR_OF_ANCHORS 5
#define RANGING_TIME 3

extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
extern TaskHandle_t uwbTaskHandle;

SemaphoreHandle_t printSemaphore;

uint32_t packets = 0;
uint32_t ANCHOR_ID = 0;

void anchor_task(void* parameters);
void print_task(void* parameters);

void appMain() {
    DEBUG_PRINT("ANCHOR ID: %d\n", ANCHOR_ID);
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(ANCHOR_ID);
    xTaskCreate(anchor_task, "anchor", configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    while(1)
        vTaskDelay(10000);
}

void anchor_task(void* parameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uwb_node_coordinates_t anchor_pos;
    uwb_set_state(TRANSMIT);
    uwb_err_code_e e;
    TickType_t t0 = xTaskGetTickCount();

    uint8_t mission_drone_counter = 20;
    while(1) {
        if(ANCHOR_ID == 0) {
            vTaskDelayUntil(&xLastWakeTime, NR_OF_ANCHORS*RANGING_TIME);
            
            anchor_pos.x = -10;
            anchor_pos.y = 1;
            anchor_pos.z = 30;
            e = uwb_do_3way_ranging_with_node(mission_drone_counter, anchor_pos);
            if(mission_drone_counter == 22)
                mission_drone_counter = 20;
            else
                mission_drone_counter++;


            if(e == UWB_SUCCESS)
                packets++;
            xSemaphoreGive(printSemaphore);
        }
        else {
            uwb_set_state(RECEIVE);
            if (xSemaphoreTake(msgReadySemaphore, 200000)) {
                if((uwb_rx_msg.src == 0)) {
                    uwb_set_state(TRANSMIT);
                    vTaskDelay(ANCHOR_ID*RANGING_TIME);

                    e = uwb_do_3way_ranging_with_node(uwb_rx_msg.dest, anchor_pos);

                    if(e == UWB_SUCCESS)
                        packets++;
                    t0 = xTaskGetTickCount();
                    xSemaphoreGive(printSemaphore);
                }
            }
        }
    }
}


void print_task(void* parameters) {
    TickType_t last_freq_print = xTaskGetTickCount();
    while(1) {
        if (xSemaphoreTake(printSemaphore, 200000 / portTICK_PERIOD_MS)) {
            if(xTaskGetTickCount() - last_freq_print > 2000) {
                last_freq_print = xTaskGetTickCount();
                DEBUG_PRINT("Rate: %d meas/sec  \n", packets / 2);
                packets = 0;
            }
        }
    }
}
