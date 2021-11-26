#ifndef UWB_API2_H
#define UWB_API_H

#define UWB_LOG_LVL_DEBUG 1
#define UWB_LOG_LVL_INFO 2
#define UWB_LOG_LVL_ERROR 3

#ifndef UWB_LOG
#define UWB_LOG(LVL, MSG)
#endif

typedef uint8_t uwb_node_id_t;
typedef uint16_t uwb_coordinate_t;
typedef uint32_t uwb_timestamp_t;

typedef enum message_type {
    UWB_RANGE_INIT_WITH_COORDS_MSG,
    UWB_RANGE_INIT_NO_COORDS_MSG,
    UWB_RANGE_RSP_MSG,
    UWB_RANGE_3WAY_FINAL_MSG,
    UWB_RANGE_4WAY_FINAL_MSG,
    UWB_RANGE_4WAY_RESULT_MSG,
    UWB_USER_MSG
} uwb_message_type_t;

typedef struct {
  uwb_coordinate_t x;
  uwb_coordinate_t y;
  uwb_coordinate_t z;
} uwb_node_coordinates_t;

typedef struct {
  uwb_timestamp_t init_msg_rcv_ts;
  uwb_timestamp_t rsp_msg_rcv_ts;
  uwb_timestamp_t final_msg_rcv_ts;
} uwb_node_timestamp_collection_t;

typedef struct {
  uint8_t* data;
  uint8_t size; /**< Size of the user payload in bytes. Must not exceed 20 bytes.*/
} uwb_user_payload_t;

typedef union {
  uwb_node_coordinates_t init_with_coords_data;
  uwb_node_timestamp_collection_t final_msg_data;
} uwb_message_payload_t;


typedef struct UWB_message
{
  node_id_t dst;
  message_type_t type;
  uwb_message_payload_t payload;
} uwb_message_t;


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
  UWB_ERROR = -6 /**< Unknown error*/
} uwb_err_code_e;

/**
 * Initialize the API
 * 
 * This function initialized the UWB API and prepares it for ranging. It must be called before any other function of this API is used.
 * @param node_id The node to initialize
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_api_init(uwb_node_id_t node_id);

uwb_err_code_e uwb_send_msg(const uwb_message_t* msg);

/**
 * Receives a ranging protocol message targeted for this node_id from the UWB transceiver.
 *
 * @param [out] Pointer to where the received message will be written to. Invalid unless return value == UWB_SUCCESS
 * @return Whether the operation succeeded.
 */
uwb_err_code_e uwb_receive_msg(uwb_message_t* rcvd_msg, TickType_t xTicksToWait);

uwb_err_code_e uwb_do_3way_ranging_with_node(uwb_node_id_t target_node_id);

uwb_err_code_e uwb_do_4way_ranging_with_node(uwb_node_id_t target_node_id, float* range_dst_mm);

#endif


// receiver task runs at prio defined UWB_RECV_TASK_PRIO
// fills queue with received messages
