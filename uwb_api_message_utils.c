#include "uwb_api_message_utils.h"
#include "deca_device_api.h"


UWB_message uwb_message_from_array(uint8_t* rx_buffer, uint8_t len) {
    struct UWB_message message = {0};
    message.ctrl = rx_buffer[0];
    message.src = rx_buffer[1];
    message.dest = rx_buffer[2];
    message.code = rx_buffer[3];
    message.data_len = len - 4 - 2;
    if((message.data_len > 0) && (message.data_len < RX_BUF_LEN - 4 - 2))
        memcpy(message.data, &rx_buffer[4], message.data_len);

    return message;
}

void uwb_array_from_message(UWB_message msg, uint8_t* array) {
    array[0] = 0xDE;
    array[1] = msg.src;
    array[2] = msg.dest;
    array[3] = msg.code;
    if (msg.data_len > 0)
        memcpy(&array[4], msg.data, msg.data_len);
}

UWB_message uwb_message_create(uint8_t dest, uint8_t src, uint8_t code, uint8_t* data, uint8_t data_len) {
    UWB_message msg = {0};
    msg.ctrl = 0xDE;
    msg.src = src;
    msg.dest = dest;
    msg.code = code;
    if (data_len > 0)
        memcpy(msg.data, &data[0], data_len);
    msg.data_len = data_len;

    return msg;
}


uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64_t var_from_8b_array(uint8_t *array, uint8_t size)
{
    uint64_t var = 0;
    for (uint8_t i = 0; i < size; i++)
        var += array[i] << (i * 8);

    return var;
}

void var_to_8b_array(uint8_t *array, uint8_t size, uint64_t var)
{
    // low index - LSB
    for (uint8_t i = 0; i < size; i++)
        array[i] = (uint8_t) ((var >> (8*i)) & 0xFF);
}
