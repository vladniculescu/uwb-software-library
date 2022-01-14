#include "system.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

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

#include "uwb_timing.h"
#include "dwm_config.h"
#include "uwb_api.h"
#include "uwb_api_internal.h"
#include "uwb_api_message_utils.h"
#include "platform_config.h"


///////////////////////////////////////////////////////  GLOBAL VARS  ///////////////////////////////////////////////////////

#define RX_INTERRUPT_MASK (DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RXPTO | DWT_INT_RFCE | DWT_INT_RFSL)
#define TX_INTERRUPT_MASK DWT_INT_TFRS

TaskHandle_t uwbTaskHandle = 0;
TaskHandle_t uwbTaskHandle_rx = 0;
SemaphoreHandle_t rxSemaphore;
QueueHandle_t msrmRcvQueue;
QueueHandle_t msgRcvQueue;

static uint64_t poll_rx_ts;
static UWB_measurement uwb_msrm = {0};

static uint8_t responder_on = 1;
static uint8_t ID = 0;


///////////////////////////////////////////////////////  COMMON  ///////////////////////////////////////////////////////
uwb_err_code_e uwb_init() {
    reset_dw();  // reset UWB module
    if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) { // init UWB module
        DEBUG_PRINT("UWB init failed\n");
        vTaskDelay(10);
        return UWB_ERROR;
    }

    dwt_configure(&config);  // send configuration to the UWB module

    dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_err_to_cb, &rx_err_to_cb);  // assign callbacks
    dwt_setinterrupt(RX_INTERRUPT_MASK, 2); 

    // Set antenna delays
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(EN_RX_AFTER_TX_DLY_UUS);  // Set expected delay for final message reception.

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
    xSemaphoreGive(rxSemaphore);

    msrmRcvQueue = xQueueCreate(1, sizeof(UWB_measurement));
    msgRcvQueue = xQueueCreate(1, sizeof(UWB_message));

    xTaskCreate(uwb_isr_task, "UWB-rx-isr",  2*configMINIMAL_STACK_SIZE, NULL, 4, &uwbTaskHandle_rx);
    vTaskDelay(10);
    enable_uwb_int();  // Enable interrupt
    platformSetLowInterferenceRadioMode();

    return UWB_SUCCESS;
}


void __attribute__((used)) EXTI11_Callback(void) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    NVIC_ClearPendingIRQ(EXTI_IRQChannel);
    EXTI_ClearITPendingBit(EXTI_LineN);

    if(responder_on == 1) {
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
    UWB_message uwb_tx_msg = {0};
    UWB_message uwb_rx_msg = {0};

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);  // Clear good RX frame event
    memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));  // Clear local RX buffer 
    
    if(cb_data->datalength <= RX_BUF_LEN)  
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);  // Read received frame into the local buffer

    uwb_rx_msg = uwb_message_from_array(rx_buffer, cb_data->datalength); // TODO check

    if(uwb_rx_msg.ctrl != 0xDE) {
        uwb_enable_rx();
        return;
    }

    if((uwb_rx_msg.dest == ID) && ((uwb_rx_msg.code == UWB_RANGE_INIT_WITH_COORDS_MSG) || (uwb_rx_msg.code == UWB_RANGE_INIT_NO_COORDS_MSG))) {
        poll_rx_ts = get_rx_timestamp_u64();  // Get poll reception timestamp.

        if(uwb_rx_msg.code == UWB_RANGE_INIT_WITH_COORDS_MSG) {
            uwb_msrm.posx = (float)*(int16_t*)&uwb_rx_msg.data[0];
            uwb_msrm.posy = (float)*(int16_t*)&uwb_rx_msg.data[2];
            uwb_msrm.posz = (float)*(int16_t*)&uwb_rx_msg.data[4];
        }
        else
            memset(&uwb_msrm, 0, sizeof(UWB_measurement));

        uint32_t resp_tx_time = (poll_rx_ts + (TX_AFTER_RX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;  // Calculate send delay time.
        dwt_setdelayedtrxtime(resp_tx_time);  // Schedule delayed transmission.

        dwt_setrxtimeout(RX_TIMEOUT_UUS);  // Set timeout for final message reception.

        uwb_tx_msg = uwb_message_create(uwb_rx_msg.src, ID, UWB_RANGE_RSP_MSG, NULL, 0);
        uwb_err_code_e e = uwb_send_msg(uwb_tx_msg, resp_tx_time, 1);

    }
    else if((uwb_rx_msg.dest == ID) && ((uwb_rx_msg.code == UWB_RANGE_3WAY_FINAL_MSG) || (uwb_rx_msg.code == UWB_RANGE_4WAY_FINAL_MSG))) {
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
        uwb_msrm.range = dist;
        uwb_msrm.src_id = uwb_rx_msg.src;
        uint32_t distance_int = (uint32_t)(dist * 1000);  // Calculate distance in mm.

        if(uwb_rx_msg.code == UWB_RANGE_4WAY_FINAL_MSG) {  // If the Initiator requests the range value back.
            uint8_t range_two_bytes[] = {(distance_int >> 8) & 0xFF,  distance_int & 0xFF};

            poll_rx_ts = get_rx_timestamp_u64();
            uint32_t resp_tx_time = (poll_rx_ts + (TX_AFTER_RX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

            uwb_tx_msg = uwb_message_create(uwb_rx_msg.src, ID, UWB_RANGE_4WAY_RESULT_MSG, &range_two_bytes[0], 2);
            uwb_err_code_e e = uwb_send_msg(uwb_tx_msg, resp_tx_time, 1);
        }
        send_msrm_to_queue(uwb_msrm);
    }
    else 
        send_msg_to_queue(uwb_rx_msg);

    uwb_enable_rx();    
}


// TX succesful callback
void tx_ok_cb(const dwt_cb_data_t *cb_data) {
}


// RX timeout callback
void rx_err_to_cb(const dwt_cb_data_t *cb_data) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
    
        // xSemaphoreGive(rxSemaphore);
        uwb_enable_rx();
        portYIELD();
}


void uwb_isr_task(void* parameters) {
    while(1) {
        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200000)) > 0) {
            dwt_isr();
        }
    }
}


void uwb_enable_rx(void) {
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}


uwb_err_code_e uwb_responder_on(void) {
    responder_on = 1;
    dwt_forcetrxoff();
    uwb_enable_rx();
    return UWB_SUCCESS;
}


uwb_err_code_e uwb_responder_off(void) {
    responder_on = 0;
    dwt_forcetrxoff();
    return UWB_SUCCESS;
}


uwb_err_code_e get_msrm_from_queue(UWB_measurement* msrm, TickType_t timeout_ms) {
    if( msrmRcvQueue != NULL ) {
        if(xQueueReceive(msrmRcvQueue, msrm, (TickType_t) timeout_ms) == pdTRUE) {
            return UWB_SUCCESS;
        }
        else return UWB_RX_TIMEOUT;
    }
    else return UWB_ERROR;
}


uwb_err_code_e send_msrm_to_queue(UWB_measurement msrm) {
    UBaseType_t nr_of_elements;
    if( msrmRcvQueue != NULL ) {
        nr_of_elements = uxQueueSpacesAvailable(msrmRcvQueue);
        if(nr_of_elements == 0)
            xQueueReset(msrmRcvQueue);
        
        if(xQueueSendToBack(msrmRcvQueue, (void*)&msrm, (TickType_t) 0) == pdTRUE) {
            return UWB_SUCCESS;
        }
    }
    return UWB_ERROR;
}


uwb_err_code_e get_msg_from_queue(UWB_message* msg, TickType_t timeout_ms) {
    if( msgRcvQueue != NULL ) {
        if(xQueueReceive(msgRcvQueue, msg, (TickType_t) timeout_ms) == pdTRUE) {
            return UWB_SUCCESS;
        }
        return UWB_RX_TIMEOUT;
    }
    return UWB_ERROR;
}


uwb_err_code_e send_msg_to_queue(UWB_message msg) {
    UBaseType_t nr_of_elements;
    if( msgRcvQueue != NULL ) {
        nr_of_elements = uxQueueSpacesAvailable(msgRcvQueue);
        if(nr_of_elements == 0)
            xQueueReset(msgRcvQueue);
        
        if(xQueueSendToBack(msgRcvQueue, (void*)&msg, (TickType_t) 0) == pdTRUE) {
            return UWB_SUCCESS;
        }
    }
    return UWB_ERROR;
}



///////////////////////////////////////////////////////  INITIATOR  ///////////////////////////////////////////////////////
uwb_err_code_e uwb_send_msg(UWB_message msg, uint32_t tx_delay, uint8_t rsp_expected) {
    uint8_t tx_msg[30];
    tx_msg[0] = 0xDE;
    tx_msg[1] = msg.src;
    tx_msg[2] = msg.dest;
    tx_msg[3] = msg.code;
    memcpy(&tx_msg[4], msg.data, msg.data_len);
    uint8_t msg_size = msg.data_len + 6;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear TX event bits
    dwt_writetxdata(msg_size, tx_msg, 0); // Zero offset in TX buffer
    dwt_writetxfctrl(msg_size, 0, 1); // Zero offset in TX buffer, ranging
    dwt_setrxtimeout(RX_TIMEOUT_UUS);

    uint8_t tx_config_mask = DWT_START_TX_IMMEDIATE + (tx_delay > 0) * DWT_START_TX_DELAYED + rsp_expected * DWT_RESPONSE_EXPECTED;
    if(tx_delay > 0) 
        dwt_setdelayedtrxtime(tx_delay);
    int32_t ret = dwt_starttx(tx_config_mask);

    if(ret < UWB_SUCCESS)
        return UWB_TX_ERROR;

    dwt_setinterrupt(DWT_INT_TFRS, 2);

    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2)) == pdFALSE) {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID); 
        if((status_reg & SYS_STATUS_TXFRS) == 0)
            dwt_forcetrxoff();
        dwt_setinterrupt(RX_INTERRUPT_MASK, 2); 
        return UWB_TX_ERROR;
    }
    dwt_setinterrupt(RX_INTERRUPT_MASK, 2);

    return UWB_SUCCESS;
}


uwb_err_code_e uwb_send_msg_wait_rsp(UWB_message msg, uint32_t tx_delay, uint8_t* rx_buf, uint8_t* rx_buf_len) {
    uwb_send_msg(msg, tx_delay, 1);
    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3)) == pdFALSE) {
        dwt_forcetrxoff();
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS); // Clear good RX frame event and TX frame
        return UWB_RX_TIMEOUT;
    }
    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);  // Read status register to check if the receive is valid

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


uwb_err_code_e uwb_do_3way_ranging_with_node(uint8_t target_id, uwb_node_coordinates_t node_pos) {
    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];
    memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));
    uint8_t rx_ret, tx_ret;

    UWB_message msg = uwb_message_create(target_id, ID, UWB_RANGE_INIT_WITH_COORDS_MSG, (uint8_t*)(&node_pos), sizeof(uwb_node_coordinates_t));
    rx_ret = uwb_send_msg_wait_rsp(msg, 0, rx_buffer, &rx_buffer_len);

    if(rx_ret != UWB_SUCCESS)
        return UWB_RX_ERROR;

    UWB_message rx_message = uwb_message_from_array(rx_buffer, rx_buffer_len); // TODO
    if((rx_message.ctrl == 0xDE) && (rx_message.dest == ID) && (rx_message.code == UWB_RANGE_RSP_MSG)) {
        uint64_t poll_tx_ts = get_tx_timestamp_u64();  // Poll transmission timestamp
        uint64_t resp_rx_ts = get_rx_timestamp_u64();  // Response transmission timestamp

        uint32_t final_tx_time = (resp_rx_ts + (TX_AFTER_RX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;   // Final message tx time
        uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;  // Final TX timestamp = tx time we programmed plus the TX antenna delay. 

        // Write all timestamps in the final message.
        msg = uwb_message_create(target_id, ID, UWB_RANGE_3WAY_FINAL_MSG, NULL, 0);
        msg.data_len = 12;

        var_to_8b_array(&msg.data[0], 4, poll_tx_ts);
        var_to_8b_array(&msg.data[4], 4, resp_rx_ts);
        var_to_8b_array(&msg.data[8], 4, final_tx_ts);

        tx_ret =  uwb_send_msg(msg, final_tx_time, 0);

        if(tx_ret != UWB_SUCCESS)
            return UWB_RX_ERROR;
        else
            return UWB_SUCCESS;    
    }
    return UWB_INVALID_RSP;
}

uwb_err_code_e uwb_do_4way_ranging_with_node(uint8_t target_id, uwb_node_coordinates_t node_pos, uint32_t* range_dst_mm) {
    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];
    memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));

    UWB_message msg = uwb_message_create(target_id, ID, UWB_RANGE_INIT_WITH_COORDS_MSG, (uint8_t*)(&node_pos), sizeof(uwb_node_coordinates_t));
    uint8_t rx_ret = uwb_send_msg_wait_rsp(msg, 0, rx_buffer, &rx_buffer_len);

    if(rx_ret != UWB_SUCCESS)
        return rx_ret;

    UWB_message rx_message = uwb_message_from_array(rx_buffer, rx_buffer_len);
    if((rx_message.ctrl == 0xDE) && (rx_message.dest == ID) && (rx_message.code == UWB_RANGE_RSP_MSG)) {
        uint64_t poll_tx_ts = get_tx_timestamp_u64();  // Poll transmission timestamp
        uint64_t resp_rx_ts = get_rx_timestamp_u64();  // Response transmission timestamp

        uint32_t final_tx_time = (resp_rx_ts + (TX_AFTER_RX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;   // Final message tx time
        uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;  // Final TX timestamp = tx time we programmed plus the TX antenna delay. 

        msg = uwb_message_create(target_id, ID, UWB_RANGE_4WAY_FINAL_MSG, NULL, 0);
        msg.data_len = 12;

        var_to_8b_array(&msg.data[0], 4, poll_tx_ts);
        var_to_8b_array(&msg.data[4], 4, resp_rx_ts);
        var_to_8b_array(&msg.data[8], 4, final_tx_ts);

        memset(rx_buffer, 0, RX_BUF_LEN*sizeof(uint8_t));
        rx_ret = uwb_send_msg_wait_rsp(msg, final_tx_time, rx_buffer, &rx_buffer_len);
        if(rx_ret != UWB_SUCCESS)
            return rx_ret;

        rx_message = uwb_message_from_array(rx_buffer, rx_buffer_len);

        if((rx_message.ctrl == 0xDE) && (rx_message.dest == ID) && (rx_message.code == UWB_RANGE_4WAY_RESULT_MSG)) {
            *range_dst_mm = (uint32_t)(rx_message.data[0] * 256 + rx_message.data[1]);
            return UWB_SUCCESS;
        }
    }
    return UWB_INVALID_RSP;
}