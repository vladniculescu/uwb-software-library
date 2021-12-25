#ifndef UWB_API_H
#define UWB_API_H

#include "deca_regs.h"
#include "dwm_utils.h"

#define RX_BUF_LEN 24
#define MSG_DATA_LEN (RX_BUF_LEN - 6)

struct UWB_message
{
    uint8_t ctrl;
    uint8_t src;
    uint8_t dest;
    uint8_t code;
    uint8_t data[MSG_DATA_LEN];
    uint8_t data_len;
};

typedef struct UWB_message UWB_message;

typedef int16_t uwb_coordinate_t;

typedef struct {
  uwb_coordinate_t x;
  uwb_coordinate_t y;
  uwb_coordinate_t z;
} uwb_node_coordinates_t;

struct UWB_measurement
{
    uint8_t src_id;
    float posx;
    float posy;
    float posz;
    float range;
};

typedef struct UWB_measurement UWB_measurement;

typedef enum message_type {
    UWB_RANGE_INIT_WITH_COORDS_MSG,
    UWB_RANGE_INIT_NO_COORDS_MSG,
    UWB_RANGE_RSP_MSG,
    UWB_RANGE_3WAY_FINAL_MSG,
    UWB_RANGE_4WAY_FINAL_MSG,
    UWB_RANGE_4WAY_RESULT_MSG,
    UWB_USER_MSG
} uwb_message_type_t;


/**
 * Possible return codes returned by the API functions
 */
typedef enum UWB_err_code
{
  UWB_SUCCESS = 0, /**< Operation succeeded.*/
  UWB_RX_TIMEOUT = -1, /**< Receive time-out occurred.*/
  UWB_RX_ERROR = -2,
  UWB_TX_ERROR = -3,
  UWB_INVALID_RSP = -4,
  UWB_INVALID_MODE = -5, /**< Trying to send data while in receive mode.*/
  UWB_ERROR = -6, /**< Unknown error*/
  UWB_TX_TIMEOUT = -7,
} uwb_err_code_e;


typedef enum UWB_err_code uwb_error;

enum states {
    RECEIVE = 0,
    TRANSMIT = 1,
    IDLE = 2
};

uwb_err_code_e uwb_api_init(uint8_t node_id);
uwb_err_code_e uwb_send_frame_wait_rsp(UWB_message msg, uint32_t tx_delay, uint8_t* rx_buf, uint8_t* rx_buf_len);
uwb_err_code_e uwb_send_frame(uint8_t* tx_msg, uint8_t msg_size, uint8_t ranging, uint32_t tx_delay);
uwb_err_code_e uwb_do_3way_ranging_with_node(uint8_t target_id, uwb_node_coordinates_t node_pos);
uwb_err_code_e uwb_do_4way_ranging_with_node(uint8_t target_id, uwb_node_coordinates_t node_pos, uint32_t* range_dst_mm);
void uwb_check_for_errors(int8_t value);
void uwb_set_state(uint8_t value);
UWB_message decode_uwb_message(uint8_t* rx_buffer, uint8_t len);

#endif 
