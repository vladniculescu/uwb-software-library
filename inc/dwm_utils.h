#ifndef DWM_UTILS_H
#define DWM_UTILS_H

void reset_dw(void);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void registerAnchorDist(float x, float y, float z, float distance, float std_dev);
uint64_t var_from_8b_array(uint8_t *array, uint8_t size);
void var_to_8b_array(uint8_t *array, uint8_t size, uint64_t var);

#endif
