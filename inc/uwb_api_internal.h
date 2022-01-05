#ifndef UWB_API_INTERNAL_H
#define UWB_API_INTERNAL_H

#include "system.h"

void rx_ok_cb(const dwt_cb_data_t *cb_data);
void tx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_err_to_cb(const dwt_cb_data_t *cb_data);
void uwb_receiver_task(void* parameters);
void uwb_isr_task(void* parameters);
void uwb_enable_rx(void);
uwb_err_code_e uwb_init();
uwb_err_code_e uwb_send_msg(UWB_message msg, uint32_t tx_delay, uint8_t rsp_expected);

uwb_err_code_e send_msrm_to_queue(UWB_measurement msrm);
uwb_err_code_e send_msg_to_queue(UWB_message msg);

#endif 
