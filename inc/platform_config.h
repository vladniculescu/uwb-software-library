#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#define GPIO_PIN_IRQ  GPIO_Pin_11
#define GPIO_PIN_RESET  GPIO_Pin_10
#define GPIO_PORT   GPIOC
#define EXTI_PortSource EXTI_PortSourceGPIOC
#define EXTI_PinSource  EXTI_PinSource11
#define EXTI_LineN    EXTI_Line11
#define EXTI_IRQChannel EXTI15_10_IRQn
#define CS_PIN DECK_GPIO_IO1

void init_io(void);
void enable_uwb_int(void);
void disable_uwb_int(void);

#endif