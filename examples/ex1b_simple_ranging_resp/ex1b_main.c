#define DEBUG_MODULE "uwb_responder"

#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uwb_api.h"

#define ANCHOR_ID 20

void print_rx_measurements(void* parameters);

void appMain() {
    uwb_api_init(ANCHOR_ID);
    uwb_responder_on();
    xTaskCreate(print_rx_measurements, "uwb-print", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    while(1)
        vTaskDelay(10000);
}


void print_rx_measurements(void* parameters) {
    UWB_measurement uwb_msrm = {0};
    uint32_t msrm_ok_cnt = 0;
    TickType_t t = xTaskGetTickCount();
    while(1) {
        if (get_msrm_from_queue(&uwb_msrm, 200000) == UWB_SUCCESS) {
            msrm_ok_cnt++;
            if(xTaskGetTickCount() - t > 1000) {
                DEBUG_PRINT("Successful in 1sec: %.1f \n", (float)msrm_ok_cnt * (float)pdMS_TO_TICKS(1000) / (float)(xTaskGetTickCount() - t));
                t = xTaskGetTickCount();
                msrm_ok_cnt = 0;
            }
        }
    }
}

