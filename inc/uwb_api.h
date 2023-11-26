#ifndef UWB_API_H
#define UWB_API_H

#include "FreeRTOS.h"
#include "deca_regs.h"

#define RX_BUF_LEN 24  /**< Maximum length of a UWB message. Excahnging larger messages might prevent the proper functionality of the API.*/
#define MSG_DATA_LEN (RX_BUF_LEN - 6)  /** Maximum data length - do not change! It is always dependent on RX_BUF_LEN.*/

/**
 * The fields of a message sent/received over UWB within this API
 */
typedef struct UWB_message {
    uint8_t ctrl;               /**< the ctrl (control) field is a constant magic number - always 0xDE.*/
    uint8_t src;                /**< the ID of the source node which sent the mesage.*/
    uint8_t dest;               /**< the ID of the target node which should receive the mesage.*/
    uint8_t code;               /**< the message type.*/
    uint8_t data[MSG_DATA_LEN]; /**< the message data field.*/
    uint8_t data_len;           /**< the length of the message data field.*/
} UWB_message;

/**
 * Structure containing the node's coordinates
 */
typedef int16_t uwb_coordinate_t;
typedef struct {
  uwb_coordinate_t x;  /**< Coordinate x in mm.*/
  uwb_coordinate_t y;  /**< Coordinate y in mm.*/
  uwb_coordinate_t z;  /**< Coordinate z in mm.*/
} uwb_node_coordinates_t;

/**
 * Structure containing the range value and the coordinates of the initiator
 */
struct UWB_measurement {
    uint8_t src_id;
    float posx;  /**< Coordinate x in m.*/
    float posy;  /**< Coordinate x in m.*/
    float posz;  /**< Coordinate x in m.*/
    float range;
};
typedef struct UWB_measurement UWB_measurement;

/**
 * Possible message types during TWR
 */
typedef enum message_type {
    UWB_RANGE_INIT_WITH_COORDS_MSG, /**< UWB ranging Initial message - signals that Initiator's coordinates are embedded in the message.*/
    UWB_RANGE_INIT_NO_COORDS_MSG,   /**< UWB ranging Initial message - signals that Initiator's coordinates are not embedded in the message.*/
    UWB_RANGE_RSP_MSG,              /**< UWB ranging Response message - Responder replies with this message to the Initial message.*/
    UWB_RANGE_3WAY_FINAL_MSG,       /**< UWB ranging Final message - Initiator sends this 3rd UWB message, telling the Responder to not send back the range value.*/
    UWB_RANGE_4WAY_FINAL_MSG,       /**< UWB ranging Final message - Initiator sends this 3rd UWB message, telling the Responder to send back the range value.*/
    UWB_RANGE_4WAY_RESULT_MSG,      /**< UWB ranging Optional message - after the Responder calcualtes the range, it sends it back to the Initiator.*/
    UWB_USER_MSG,                   /**< UWB general purpose message.*/
    UWB_ACK_MSG,                    /**< UWB message requiring acknowledge.*/
    UWB_ACK_RES_MSG                 /**< UWB acknowledge message.*/
} uwb_message_type_t;

/**
 * Possible return codes returned by the API functions
 */
typedef enum UWB_err_code {
  UWB_SUCCESS = 0,      /**< Operation succeeded.*/
  UWB_RX_TIMEOUT = -1,  /**< Receive time-out occurred.*/
  UWB_RX_ERROR = -2,    /**< Receive failed.*/
  UWB_TX_ERROR = -3,    /**< Transmit failed.*/
  UWB_INVALID_RSP = -4, /**< Unexpected response.*/
  UWB_ERROR = -5        /**< Unknown error*/
} uwb_err_code_e;
typedef enum UWB_err_code uwb_error;


/* API functions available to the user */

/**
 * Initialize the API
 * 
 * This function initialized the UWB API and prepares it for ranging. It must be called before any other function of this API is used.
 * @param node_id The node to initialize
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_api_init(uint8_t node_id);


/**
 * Send UWB message
 * 
 * This function sends a UWB message from the current node to the target node.
 * Note: to use this function, the responder mode has to be off, by calling uwb_responder_off();
 * @param msg the message structure
 * @param tx_delay timestamp for the UWB module scheduled transmission
 * @param rsp_expected programs the UWB module to automatically switch on the receiver after the transmission is completed
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_send_msg(UWB_message msg, uint32_t tx_delay, uint8_t rsp_expected);


/**
 * Send UWB message and wait for a response message
 * 
 * This function sends a UWB message from the current node to the target node and waits for a reply from the target node.
 * Note: to use this function, the responder mode has to be off, by calling uwb_responder_off();
 * @param msg the message structure
 * @param tx_delay timestamp for the UWB module scheduled transmission
 * @param rx_buf indicates where to store the response message
 * @param rx_buf_len size of the received message
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_send_msg_wait_rsp(UWB_message msg, uint32_t tx_delay, uint8_t* rx_buf, uint8_t* rx_buf_len);


/**
 * Perform 3-message ranging.
 * 
 * This function initializes ranging by sending the Initial message, waiting for the Response message and then sending again the Final message.
 * After the execution of this function is completed, the range value can be computed in the other node.
 * Note: to use this function, the responder mode has to be off, by calling uwb_responder_off();
 * @param target_id the is of the node to do ranging with
 * @param uwb_node_coordinates_t the coordinates of the current node
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_do_3way_ranging_with_node(uint8_t target_id, uwb_node_coordinates_t node_pos);


/**
 * Perform 4-message ranging.
 * 
 * This function initializes ranging by sending the Initial message, waiting for the Response message and then sending again the Final message.
 * The other node (the target node) computes the range and sends it back to this node via the Optional message. After this function is executed,
 * both nodes will have the range value.
 * Note: to use this function, the responder mode has to be off, by calling uwb_responder_off();
 * @param target_id the is of the node to do ranging with
 * @param uwb_node_coordinates_t the coordinates of the current node
 * @param range_dst_mm pointer to where to store the range value [mm]
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_do_4way_ranging_with_node(uint8_t target_id, uwb_node_coordinates_t node_pos, uint32_t* range_dst_mm);


/**
 * Fetch the received UWB message from the receive queue.
 * 
 * This function fetches a UWB message from the receive queue. The size of the queue is 1, to reduce the memory occupancy.
 * Therefore, this function will always read the last received UWB message. The function is blocking, which means that the 
 * task will not advance before a message is received or before time-out.
 * Note: to receive messages, the responder mode has to be on, by calling uwb_responder_on();
 * @param msg pointer to where the message has to be copied from the queue
 * @param timeout_ms wait time-out: if no message is received fro this time, return from the function.
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e get_msg_from_queue(UWB_message* msg, TickType_t timeout_ms);


/**
 * Fetch the received UWB measurement from the receive queue.
 * 
 * This function fetches a UWB measurement from the receive queue. The size of the queue is 1, to reduce the memory occupancy.
 * Therefore, this function will always read the last received UWB measurement. The function is blocking, which means that the 
 * task will not advance before a measurement is received or before time-out.
 * Note: to receive measurements, the responder mode has to be on, by calling uwb_responder_on();
 * @param msrm pointer to where the measurement has to be copied from the queue
 * @param timeout_ms wait time-out: if no measurement is received fro this time, return from the function.
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e get_msrm_from_queue(UWB_measurement* msrm, TickType_t timeout_ms);


/**
 * Enable responder mode
 * 
 * This function is turning on the receive mode for the UWB module which starts listening to messages.
 * Once the responder has been enabled, it can only be disabled by calling uwb_responder_off();

 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_responder_on(void);


/**
 * Disable responder mode
 * 
 * This function is turning off the receive mode and therefore the node can operate as Initiator.
 * Once the responder has been disabled, it can only be re-enabled by calling uwb_responder_on();
 * @return Whether the operation succeeded.
 **/
uwb_err_code_e uwb_responder_off(void);

#endif 
