# uwb-software-library
The USL (Ultra-wideband Software Library) was develop to provide out-of-the-box functionalities for the Decawave DW1000 module when used in combination with the commercial drone Bitcraze Crazyflie.
However, with minimal modifications, USL can be easily adjusted to work under any Free-RTOS system.

The functionalities provided by this library are:
1) Perform ranging with another UWB node and obtain the distance measurement in the initiator node (Double-Sided TWR with 3 messages)
2) Perform ranging with another UWB node and obtain the distance measurement in both UWB nodes (Double-Sided TWR with 4 messages - the 4th messages is simply a data message communicating the distance to the initiator node)
3) Send an UWB message
4) Send an UWB data packet

We mention that using these functionalities is very straightforward and typically requires one line of code. The examples folder shows how to use the basic ranging functionalities to obtain distance measurements (ex1a and ex1b) and also how to enable a Crazyflie drone to localize itself using an anchor network of UWB nodes.



## Flashing Instructions
The USL was tested with the following commit of the crazyflie-firmware:
- Switch to the commit *f7df7f334be57acd7b18fe0760fb99ed828597aa*, using the command: 
`git checkout f7df7f334be57acd7b18fe0760fb99ed828597aa`
