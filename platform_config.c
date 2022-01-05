#include "deck.h"
#include "FreeRTOS.h"
#include "platform_config.h"
#include "debug.h"

static EXTI_InitTypeDef EXTI_InitStructure;
static GPIO_InitTypeDef GPIO_InitStructure;
static NVIC_InitTypeDef NVIC_InitStructure;

void init_io(void)
{
    initUsecTimer();
    SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

    EXTI_InitStructure.EXTI_Line = EXTI_LineN;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    pinMode(DECK_GPIO_RX1,INPUT);

    // Init reset output
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RESET;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT, &GPIO_InitStructure);
}

void enable_uwb_int(void)
{
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void disable_uwb_int(void)
{
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void reset_dw(void)
{
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));
}