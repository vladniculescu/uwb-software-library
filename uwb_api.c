#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "system.h"
#include "debug.h"
#include "deck.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "log.h"
#include "param.h"
#include "estimator_kalman.h"
#include "estimator.h"
#include "platform.h"

#include "deca_device_api.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "uwb_timing.h"
#include "dwm_config.h"
#include "uwb_api.h"
#include "platform_config.h"

///////////////////////////////////////////////////////  GLOBAL VARS  ///////////////////////////////////////////////////////

// EXTERN
TaskHandle_t uwbTaskHandle = 0;
TaskHandle_t uwbTaskHandle_rx = 0;
SemaphoreHandle_t rxSemaphore;
SemaphoreHandle_t msgReadySemaphore;
SemaphoreHandle_t msrmReadySemaphore;
SemaphoreHandle_t isrSemaphore;

// FOR THE RESPONDER
static uint64_t poll_rx_ts;
UWB_message uwb_rx_msg = {0};
static TickType_t last_time_isr = 0;
UWB_measurement last_range_msrm = {0};

// FOR THE INITIATOR


//COMMON
static uint8_t state = 0;
static uint8_t ID = 0;

// FUNCTION HEADERS
void rx_ok_cb(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
void uwb_receiver_task(void* parameters);
void uwb_isr_task(void* parameters);
uwb_err_code_e uwb_init();

UWB_message decode_uwb_message(uint8_t* rx_buffer, uint8_t len) {
    struct UWB_message message = {0};
    message.ctrl = rx_buffer[0];
    message.src = rx_buffer[1];
    message.dest = rx_buffer[2];
    message.code = rx_buffer[3];
    message.data_len = len - 4 - 2;
    if(message.data_len > 0) 
        memcpy(message.data, &rx_buffer[4], message.data_len);

    return message;
}

///////////////////////////////////////////////////////  COMMON  ///////////////////////////////////////////////////////
uwb_err_code_e uwb_init() {
    reset_dw();  // reset UWB module
    if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) { // init UWB module
        DEBUG_PRINT("UWB init failed\n");
        vTaskDelay(10);
        return UWB_ERROR;
    }

    dwt_configure(&config);  // send configuration to the UWB module

    dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);  // assign callbacks
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RXPTO | DWT_INT_RFCE | DWT_INT_RFSL, 2); 

    // Set antenna delays
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setleds(DWT_LEDS_ENABLE);  // Enable LEDs for visual feedback

    dwt_setsmarttxpower(0);  // Smart power management disabled to maximize TX range
    dwt_txconfig_t configTX;
    configTX.power = 0x1F1F1F1F;  // Command max power
    configTX.PGdly = 0xC2;  // Command max power
    dwt_configuretxrf(&configTX);

    return UWB_SUCCESS;
}


uwb_err_code_e uwb_api_init(uint8_t node_id) {
    ID = node_id;
    init_io();

    pinMode(CS_PIN, OUTPUT);  // Init CS pin

    spiBegin();
    set_spi_speed(0);
    if(uwb_init() != UWB_SUCCESS)
        return UWB_ERROR;
    set_spi_speed(1);

    vSemaphoreCreateBinary(rxSemaphore);
    vSemaphoreCreateBinary(msgReadySemaphore);
    vSemaphoreCreateBinary(msrmReadySemaphore);
    vSemaphoreCreateBinary(isrSemaphore);
    xSemaphoreGive(rxSemaphore);
    xSemaphoreGive(msgReadySemaphore);
    xSemaphoreGive(msrmReadySemaphore);
    xSemaphoreGive(isrSemaphore);

    enable_uwb_int();  // Enable interrupt

    // xTaskCreate(uwb_receiver_task, "UWB-rx-en", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    xTaskCreate(uwb_isr_task, "UWB-rx-isr",  configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle_rx);

    platformSetLowInterferenceRadioMode();

    return UWB_SUCCESS;
}


void uwb_set_state(uint8_t value) { //TODO - set mode; get rid of this, use rx en / rx disable
    switch(value) 
    {
        case RECEIVE:
            state = RECEIVE;
            dwt_forcetrxoff();
            dwt_setrxtimeout(0);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            break; 
            
        case TRANSMIT:
            state = TRANSMIT;
            dwt_forcetrxoff();
            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
            break; 

        case IDLE:
            state = IDLE;
            dwt_forcetrxoff();
            break;
    }
}


void __attribute__((used)) EXTI11_Callback(void) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    NVIC_ClearPendingIRQ(EXTI_IRQChannel);
    EXTI_ClearITPendingBit(EXTI_LineN);

    if(state==RECEIVE) {
        vTaskNotifyGiveFromISR(uwbTaskHandle_rx, &xHigherPriorityTaskWoken); 
    }
    else {
        vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken); 
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


///////////////////////////////////////////////////////  RESPONDER  ///////////////////////////////////////////////////////

// RX ok callback
void rx_ok_cb(const dwt_cb_data_t *cb_data) {  
    uint8_t rx_buffer[RX_BUF_LEN];

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);  // Clear good RX frame event
    memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));  // Clear local RX buffer 
    
    if(cb_data->datalength <= RX_BUF_LEN)  
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);  // Read received frame into the local buffer

    uwb_rx_msg = decode_uwb_message(rx_buffer, cb_data->datalength);

    if(uwb_rx_msg.ctrl == 0xDE)
        xSemaphoreGive(msgReadySemaphore);

    // DEBUG_PRINT("%d dest %d, src %d, code %d \n", xTaskGetTickCount() % 10000, uwb_rx_msg.dest, uwb_rx_msg.src, uwb_rx_msg.code);

    if((uwb_rx_msg.ctrl == 0xDE) && (uwb_rx_msg.dest == ID) && (uwb_rx_msg.code == UWB_RANGE_INIT_NO_COORDS_MSG)) {
            poll_rx_ts = get_rx_timestamp_u64();  // Get poll reception timestamp.

            uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;  // Calculate send delay time.
            dwt_setdelayedtrxtime(resp_tx_time);  // Schedule delayed transmission.

            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);  // Set expected delay for final message reception.
            dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);  // Set timeout for final message reception.

            uint8_t tx_resp_msg[] = {0xDE, ID, uwb_rx_msg.src, UWB_RANGE_RSP_MSG, 0, 0};  // Create message
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);  // Write message to the UWB module
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);  // Zero offset in TX buffer.
            int32_t ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
            if(ret < UWB_SUCCESS) {
                // xSemaphoreGive(rxSemaphore);
                dwt_forcetrxoff();
                uwb_enable_rx();
                portYIELD();
            }
            dwt_setinterrupt(DWT_INT_TFRS, 2); 
            if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3)) == pdFALSE)
            {
                uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID); 
                if((status_reg & SYS_STATUS_TXFRS) == 0)
                    dwt_forcetrxoff();
                    uwb_enable_rx();
                    portYIELD();
            }
            dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RXPTO | DWT_INT_RFCE | DWT_INT_RFSL, 2);
    }
    else if((uwb_rx_msg.ctrl == 0xDE) && (uwb_rx_msg.dest == ID) && ((uwb_rx_msg.code == UWB_RANGE_3WAY_FINAL_MSG) || (uwb_rx_msg.code == UWB_RANGE_4WAY_FINAL_MSG))) {
            dwt_setleds(DWT_LEDS_ENABLE);  // Enable LEDs for visual feedback.

            uint64_t resp_tx_ts = get_tx_timestamp_u64();  // Retrieve response transmission timestamp.
            uint64_t final_rx_ts = get_rx_timestamp_u64();  // Retrieve final reception timestamps.

            uint32_t poll_tx_ts = (uint32_t) var_from_8b_array(&uwb_rx_msg.data[0], 4);
            uint32_t resp_rx_ts = (uint32_t) var_from_8b_array(&uwb_rx_msg.data[4], 4);
            uint32_t final_tx_ts = (uint32_t) var_from_8b_array(&uwb_rx_msg.data[8], 4);

            // Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped.
            uint32_t poll_rx_ts_32 = (uint32_t)poll_rx_ts;
            uint32_t resp_tx_ts_32 = (uint32_t)resp_tx_ts;
            uint32_t final_rx_ts_32 = (uint32_t)final_rx_ts;
            int64_t Ra = (resp_rx_ts - poll_tx_ts);
            int64_t Rb = (final_rx_ts_32 - resp_tx_ts_32);
            int64_t Da = (final_tx_ts - resp_rx_ts);
            int64_t Db = (resp_tx_ts_32 - poll_rx_ts_32);
            double nom = (double)(Ra * Rb - Da * Db);
            double denom = (double)(Ra + Rb + Da + Db);
            double tof_dtu = nom / denom;
            double tof = tof_dtu * DWT_TIME_UNITS;

            float dist = (float)(tof * SPEED_OF_LIGHT);  // Calculate distance in meters.
            last_range_msrm.range = dist;
            last_range_msrm.src_id = uwb_rx_msg.src;
            uint32_t distance_int = (uint32_t)(dist * 1000);  // Calculate distance in mm.
            xSemaphoreGive(msrmReadySemaphore);

            if(uwb_rx_msg.code == UWB_RANGE_4WAY_FINAL_MSG) {  // If the Initiator requests the range value back.
                uint8_t data_message[] = {0xDE, ID, uwb_rx_msg.src, UWB_RANGE_4WAY_RESULT_MSG,0,0,0,0};  // Build the data message, that should contain the range.
                // Split the range in two bytes and load it into the data message.
                data_message[4] = (distance_int >> 8) & 0xFF; 
                data_message[5] = distance_int & 0xFF;
                // Start delayed transmission.
                poll_rx_ts = get_rx_timestamp_u64();
                uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);
                dwt_writetxdata(sizeof(data_message), data_message, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(data_message), 0, 1); /* Zero offset in TX buffer, ranging. */
                int ret = dwt_starttx(DWT_START_TX_DELAYED);
                if(ret < UWB_SUCCESS) {
                    // xSemaphoreGive(rxSemaphore);
                    dwt_forcetrxoff();
                    uwb_enable_rx();
                    portYIELD();
                }

                dwt_setinterrupt(DWT_INT_TFRS, 2); 
                if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2)) == pdFALSE)
                {
                    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID); 
                    if((status_reg & SYS_STATUS_TXFRS) == 0)
                        dwt_forcetrxoff();
                        uwb_enable_rx();
                        portYIELD();
                }
                dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RXPTO | DWT_INT_RFCE | DWT_INT_RFSL, 2);
            }

            // xSemaphoreGive(rxSemaphore);
            uwb_enable_rx();
            portYIELD();
    }
    else {
        // xSemaphoreGive(rxSemaphore);
        uwb_enable_rx();
        portYIELD();
    }
}


// TX succesful callback
void tx_conf_cb(const dwt_cb_data_t *cb_data) {
}


// RX timeout callback
void rx_to_cb(const dwt_cb_data_t *cb_data) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
    
        // xSemaphoreGive(rxSemaphore);
        uwb_enable_rx();
        portYIELD();
}


// RX error callback
void rx_err_cb(const dwt_cb_data_t *cb_data) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
    
        // xSemaphoreGive(rxSemaphore);
        uwb_enable_rx();
        portYIELD();
}


void uwb_isr_task(void* parameters) {
    while(1) {
        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200000)) > 0) {
            if(state == RECEIVE) {
                last_time_isr = xTaskGetTickCount();
                dwt_isr();
            }   
        }
    }
}


void uwb_receiver_task(void* parameters) {
    while(1) {
        if(xSemaphoreTake(rxSemaphore, 10000 / portTICK_PERIOD_MS)) {
            if(state==RECEIVE) {
                dwt_setrxtimeout(0);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
    }
}


void uwb_enable_rx(void) {
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}


///////////////////////////////////////////////////////  INITIATOR  ///////////////////////////////////////////////////////

uwb_err_code_e uwb_send_frame_wait_rsp(uint8_t* tx_msg, uint8_t msg_size, uint32_t tx_delay, uint8_t* rx_buf, uint8_t* rx_buf_len) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear TX event bits
    dwt_writetxdata(msg_size, tx_msg, 0); // Zero offset in TX buffer
    dwt_writetxfctrl(msg_size, 0, 1); // Zero offset in TX buffer, ranging

    int32_t ret, status_reg;
    if(tx_delay > 0) { // Check if delayed TX is desired
        dwt_setdelayedtrxtime(tx_delay);
        ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);  // TX starts - response expected
    }
    else
        ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);  // TX starts - response expected

    if(ret < UWB_SUCCESS)
        return UWB_TX_ERROR;

    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3)) == pdFALSE) {
        status_reg = dwt_read32bitreg(SYS_STATUS_ID); 
        if((status_reg & SYS_STATUS_TXFRS) == 0)
            dwt_forcetrxoff();
            return UWB_TX_ERROR;
    }
    else
        status_reg = dwt_read32bitreg(SYS_STATUS_ID);  // Read status register to check if the receive is valid

    if(status_reg & SYS_STATUS_RXFCG) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS); // Clear good RX frame event and TX frame
        uint8_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN)
        dwt_readrxdata(rx_buf, frame_len, 0);  // Read frame
        *rx_buf_len = frame_len;
        return UWB_SUCCESS;
    }
    else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR); // Clear RX error/timeout
        dwt_rxreset(); //reset RX
        return UWB_RX_ERROR;
    }
}


uwb_err_code_e uwb_send_frame(uint8_t* tx_msg, uint8_t msg_size, uint8_t ranging, uint32_t tx_delay) {
    uint8_t ret = 0;
    dwt_writetxdata(msg_size, tx_msg, 0); // Zero offset in TX buffer
    dwt_writetxfctrl(msg_size, 0, ranging); // Zero offset in TX buffer, ranging

    dwt_setinterrupt(DWT_INT_TFRS, 2); 

    if(tx_delay > 0) {
        dwt_setdelayedtrxtime(tx_delay);
        ret = dwt_starttx(DWT_START_TX_DELAYED);
    }
    else
        ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2)) == pdFALSE) {
        uint32_t status = dwt_read32bitreg(SYS_STATUS_ID); 
        if((status & SYS_STATUS_TXFRS) == 0)
            dwt_forcetrxoff();
            dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RXPTO | DWT_INT_RFCE | DWT_INT_RFSL, 2); 
            return UWB_TX_ERROR;
    }
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RXPTO | DWT_INT_RFCE | DWT_INT_RFSL, 2); 

    if(ret < UWB_SUCCESS)
        return UWB_TX_ERROR;
    else
        return UWB_SUCCESS;
}


uwb_err_code_e uwb_do_3way_ranging_with_node(uint8_t target_id, uint8_t* node_pos) {
    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];
    memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));
    uint8_t rx_ret, tx_ret;
    if(node_pos[0]) {
        uint8_t x_m = (node_pos[1] >> 8) & 0xFF;
        uint8_t x_l = node_pos[2] & 0xFF;
        uint8_t y_m = (node_pos[3] >> 8) & 0xFF;
        uint8_t y_l = node_pos[4] & 0xFF;
        uint8_t z_m = (node_pos[5] >> 8) & 0xFF;
        uint8_t z_l = node_pos[6] & 0xFF;
        uint8_t tx_poll_msg[] = {0xDE, ID, target_id, UWB_RANGE_INIT_NO_COORDS_MSG, x_m, x_l, y_m, y_l, z_m, z_l, 0, 0};
        rx_ret = uwb_send_frame_wait_rsp(tx_poll_msg, sizeof(tx_poll_msg), 0, rx_buffer, &rx_buffer_len);
    }
    else {
        uint8_t tx_poll_msg[] = {0xDE, ID, target_id, UWB_RANGE_INIT_NO_COORDS_MSG, 0, 0};
        rx_ret = uwb_send_frame_wait_rsp(tx_poll_msg, sizeof(tx_poll_msg), 0, rx_buffer, &rx_buffer_len);
    }

    if(rx_ret != UWB_SUCCESS)
        return UWB_RX_ERROR;

    UWB_message rx_message = decode_uwb_message(rx_buffer, rx_buffer_len);
    if((rx_message.ctrl == 0xDE) && (rx_message.dest == ID) && (rx_message.code == UWB_RANGE_RSP_MSG)) {
        uint64_t poll_tx_ts = get_tx_timestamp_u64();  // Poll transmission timestamp
        uint64_t resp_rx_ts = get_rx_timestamp_u64();  // Response transmission timestamp

        uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;   // Final message tx time
        uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;  // Final TX timestamp = tx time we programmed plus the TX antenna delay. 

        // Write all timestamps in the final message.
        uint8_t tx_final_msg[] = {0xDE, ID, target_id, UWB_RANGE_3WAY_FINAL_MSG, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        var_to_8b_array(&tx_final_msg[4], 4, poll_tx_ts);
        var_to_8b_array(&tx_final_msg[8], 4, resp_rx_ts);
        var_to_8b_array(&tx_final_msg[12], 4, final_tx_ts);

        tx_ret = uwb_send_frame(tx_final_msg, sizeof(tx_final_msg), 1, final_tx_time);
        if(tx_ret != UWB_SUCCESS)
            return UWB_RX_ERROR;
        else
            return UWB_SUCCESS;    
    }
    return UWB_INVALID_RSP;
}

uwb_err_code_e uwb_do_4way_ranging_with_node(uint8_t target_id, uint8_t* node_pos, float* range_dst_mm) {
    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];
    memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));
    uint8_t rx_ret, tx_ret;
    if(node_pos[0]) {
        uint8_t x_m = (node_pos[1] >> 8) & 0xFF;
        uint8_t x_l = node_pos[2] & 0xFF;
        uint8_t y_m = (node_pos[3] >> 8) & 0xFF;
        uint8_t y_l = node_pos[4] & 0xFF;
        uint8_t z_m = (node_pos[5] >> 8) & 0xFF;
        uint8_t z_l = node_pos[6] & 0xFF;
        uint8_t tx_poll_msg[] = {0xDE, ID, target_id, UWB_RANGE_INIT_NO_COORDS_MSG, x_m, x_l, y_m, y_l, z_m, z_l, 0, 0};
        rx_ret = uwb_send_frame_wait_rsp(tx_poll_msg, sizeof(tx_poll_msg), 0, rx_buffer, &rx_buffer_len);
    }
    else {
        uint8_t tx_poll_msg[] = {0xDE, ID, target_id, UWB_RANGE_INIT_NO_COORDS_MSG, 0, 0};
        rx_ret = uwb_send_frame_wait_rsp(tx_poll_msg, sizeof(tx_poll_msg), 0, rx_buffer, &rx_buffer_len);
    }

    if(rx_ret != UWB_SUCCESS)
        return rx_ret;

    UWB_message rx_message = decode_uwb_message(rx_buffer, rx_buffer_len);
    if((rx_message.ctrl == 0xDE) && (rx_message.dest == ID) && (rx_message.code == UWB_RANGE_RSP_MSG)) {
        uint64_t poll_tx_ts = get_tx_timestamp_u64();  // Poll transmission timestamp
        uint64_t resp_rx_ts = get_rx_timestamp_u64();  // Response transmission timestamp

        uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;   // Final message tx time
        uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;  // Final TX timestamp = tx time we programmed plus the TX antenna delay. 

        // Write all timestamps in the final message.
        uint8_t tx_final_msg[] = {0xDE, ID, target_id, UWB_RANGE_4WAY_FINAL_MSG, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        var_to_8b_array(&tx_final_msg[4], 4, poll_tx_ts);
        var_to_8b_array(&tx_final_msg[8], 4, resp_rx_ts);
        var_to_8b_array(&tx_final_msg[12], 4, final_tx_ts);

        memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));
        rx_ret = uwb_send_frame_wait_rsp(tx_final_msg, sizeof(tx_final_msg), final_tx_time, rx_buffer, &rx_buffer_len);
        if(rx_ret != UWB_SUCCESS)
            return rx_ret;

        rx_message = decode_uwb_message(rx_buffer, rx_buffer_len);

        if((rx_message.ctrl == 0xDE) && (rx_message.dest == ID) && (rx_message.code == UWB_RANGE_4WAY_RESULT_MSG)) {
            float data = (float)(rx_message.data[0] * 256 + rx_message.data[1]);
            *range_dst_mm = data;
            return UWB_SUCCESS;
        }

    }
    return UWB_INVALID_RSP;
}