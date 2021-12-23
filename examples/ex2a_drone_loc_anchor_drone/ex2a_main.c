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

extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
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


// void EnableTimerInterrupt()
// {
//     NVIC_InitTypeDef nvicStructure;
//     nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
//     nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
//     nvicStructure.NVIC_IRQChannelSubPriority = 1;
//     nvicStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvicStructure);
// }

// void timer5_init() {
//     // Enable the peripheral clock
//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

//     // Configure the timebase
//     TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
//     TIM_TimeBaseInitStructure.TIM_Prescaler = 1;
//     TIM_TimeBaseInitStructure.TIM_Period = 35999;
//     TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

//     // That last function caused the UIF flag to get set. Clear it.
//     TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

//     // Configure so that the interrupt flag is only set upon overflow
//     TIM_UpdateRequestConfig(TIM6, TIM_UpdateSource_Regular);

//     // Enable the TIM5 Update Interrupt type
//     TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
// }

void anchor_task(void* parameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uwb_node_coordinates_t anchor_pos;
    uwb_set_state(TRANSMIT);
    uwb_err_code_e e;

    uint8_t md_counter = MD_FIRST_ID;

    // timer5_init();
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
            uwb_set_state(RECEIVE);
            if (xSemaphoreTake(msgReadySemaphore, 200000)) {
                if((uwb_rx_msg.src == 0)) {
                    uwb_set_state(TRANSMIT);
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
