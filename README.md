
# uwb-software-library
## Introduction to the USL
The USL (Ultra-wideband Software Library) was develop to provide out-of-the-box functionalities for the Decawave DW1000 module when used in combination with the commercial drone Bitcraze Crazyflie.
However, with minimal modifications, USL can be easily adjusted to work under any Free-RTOS system.

The functionalities provided by this library are:
1) Perform ranging with another UWB node and obtain the distance measurement in the initiator node (Double-Sided TWR with 3 messages)
2) Perform ranging with another UWB node and obtain the distance measurement in both UWB nodes (Double-Sided TWR with 4 messages - the 4th messages is simply a data message communicating the distance to the initiator node)
3) Send an UWB message
4) Send an UWB data packet

We mention that using these functionalities is very straightforward and typically requires one line of code. The examples folder shows how to use the basic ranging functionalities to obtain distance measurements (ex1a and ex1b) and also how to enable a Crazyflie drone to localize itself using an anchor network of UWB nodes.
It is important to mention that a node using this UWB (i.e., a drone) can only perform ranging with another node that is also using USL. Out of the box, this library was designed to enable Crazyflie drones to perform ranging measurements in between them. However, the ranging also works if the drone is stationary and therefore a drone could also act as a static anchor.

## Setup Instructions
1. Get the Crazyflie Firmware from [here](https://github.com/bitcraze/crazyflie-firmware).
2. The USL was tested with the following commit of the crazyflie-firmware: *f7df7f334be57acd7b18fe0760fb99ed828597aa*
Switch to the recommended commit:
`git checkout f7df7f334be57acd7b18fe0760fb99ed828597aa`
3. The existing UWB driver of the crazyflie firmware uses the same interrupt function as the USL. To avoid having the same function defined twice, use one of the two fixes:
a. LOCODECK_USE_ALT_PINS in the crazyflie firmware
b. Navigate to `crazyflie-firmware/src/deck/drivers/src/locodeck.c` and comment out lines 427-441
4. In  `crazyflie-firmware/`open the `Makefile` and comment out the line 323 (CFLAGS += -Werror)

## Example 1: Basic ranging
In `uwb-api/examples/` `ex1a_simple_ranging_ini` and `ex1a_simple_ranging_resp` belong to Example 1. Since in any UAB communication, one node initiates the ranging process and the other node responds and cooperates throughout ranging, two independent applications are required. `ex1a_simple_ranging_ini`should be flashed in the drone that always initiates ranging, while `ex1a_simple_ranging_resp` should flash the responder.
Note that in each folder there is a Makefile, where the variables `CRAZYFLIE_BASE` and
`UWB_API_BASE` specify the path of the Crazyflie fiwmware and the UWB, respectively. So the example folder can be moved anywhere outside the API as long as the paths are correctly updated.
To flash the exmples:
`cd examples/ex1a_simple_ranging_ini`
`make all`
`make cload` -- with the drone in bootloader mode

Note: for a more detailed description of the API functions, check `inc/uwb_api.h`
