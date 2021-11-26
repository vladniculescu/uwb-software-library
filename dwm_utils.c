#include "FreeRTOS.h"
#include "task.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "deca_device_api.h"
#include "dwm_utils.h"
#include "platform_config.h"


void reset_dw(void)
{
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));
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

void registerAnchorDist(float x, float y, float z, float distance, float std_dev) {
    if (distance < 100.0f && distance > 0.1f) {
        distanceMeasurement_t dist;
        dist.distance = distance;
        dist.x = x;
        dist.y = y;
        dist.z = z;
        dist.stdDev = std_dev;
        estimatorEnqueueDistance(&dist);
    }
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
