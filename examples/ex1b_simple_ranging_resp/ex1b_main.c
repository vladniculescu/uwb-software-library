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
extern SemaphoreHandle_t msgReadySemaphore;
extern SemaphoreHandle_t msrmReadySemaphore;
extern UWB_message uwb_rx_msg;
extern float last_range_msrm;

void print_rx_measurements(void* parameters);

void appMain()
{
    uwb_api_init(ANCHOR_ID);
    uwb_set_state(RECEIVE);
    xTaskCreate(print_rx_measurements, "uwb-print", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

    while(1)
        vTaskDelay(10000);
}


void print_rx_measurements(void* parameters) {
    while(1) {
        if (xSemaphoreTake(msrmReadySemaphore, 200000 / portTICK_PERIOD_MS)) {
            // DEBUG_PRINT("Distance: %.3f \n", last_range_msrm);
        }
    }
}

