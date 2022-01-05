#ifndef UWB_API_MESSAGE_UTILS_H
#define UWB_API_MESSAGE_UTILS_H

#include "uwb_api.h"
#include <string.h>
#include <stdlib.h>
#include "system.h"


UWB_message uwb_message_from_array(uint8_t* rx_buffer, uint8_t len);
UWB_message uwb_message_create(uint8_t dest, uint8_t src, uint8_t code, uint8_t* data, uint8_t data_len);
void uwb_array_from_message(UWB_message msg, uint8_t* array);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
uint64_t var_from_8b_array(uint8_t *array, uint8_t size);
void var_to_8b_array(uint8_t *array, uint8_t size, uint64_t var);

#endif