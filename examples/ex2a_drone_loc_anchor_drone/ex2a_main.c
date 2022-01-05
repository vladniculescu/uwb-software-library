#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uwb_api.h"
#include "stm32f4xx_tim.h"
#include "config_params.h"

#define DEBUG_MODULE "AD"

extern TaskHandle_t uwbTaskHandle;

SemaphoreHandle_t printSemaphore;

int32_t packets = 0;

void anchor_task(void* parameters);
void print_task(void* parameters);

void appMain() {
    DEBUG_PRINT("ANCHOR ID: %d\n", ANCHOR_ID);
    vSemaphoreCreateBinary(printSemaphore);
    xSemaphoreGive(printSemaphore);

    uwb_api_init(ANCHOR_ID);
    xTaskCreate(anchor_task, "anchor", 2*configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle);
    xTaskCreate(print_task, "printer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    while(1)
        vTaskDelay(10000);
}


void anchor_task(void* parameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uwb_node_coordinates_t anchor_pos;
    uwb_responder_off();
    uwb_err_code_e e;

    uint8_t md_counter = MD_FIRST_ID;
    UWB_message uwb_rx_msg = {0};

    while(1) {
        if(ANCHOR_ID == 0) {
            vTaskDelayUntil(&xLastWakeTime, NR_OF_ANCHORS*RANGING_TIME);
            e = uwb_do_3way_ranging_with_node(md_counter, anchor_pos);
            if(md_counter == (MD_FIRST_ID + NR_OF_MD - 1))
                md_counter = MD_FIRST_ID;
            else
                md_counter++;

            if(e == UWB_SUCCESS)
                packets++;
            xSemaphoreGive(printSemaphore);
        }
        else {
            uwb_responder_on();
            if (get_msg_from_queue(&uwb_rx_msg, 200000) == UWB_SUCCESS) {
                if((uwb_rx_msg.src == 0)) {
                    uwb_responder_off();
                    vTaskDelay(ANCHOR_ID*RANGING_TIME);
                    e = uwb_do_3way_ranging_with_node(uwb_rx_msg.dest, anchor_pos);
                    if(e == UWB_SUCCESS)
                        packets++;
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
