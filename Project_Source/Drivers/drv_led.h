#ifndef __DRV_LED_H__
#define __DRV_LED_H__
#include "ciu32f003_std.h"
#include "ciu32f003_std_rcc.h"
#include "ciu32f003_std_gpio.h"
#define LED1_CLOCK RCC_PERIPH_CLK_GPIOB
#define LED1_PORT  GPIOB
#define LED1_PIN   GPIO_PIN_1
void drv_led_init(void);
void drv_led_on(void);
void drv_led_off(void);
void drv_led_toggle(void);
#endif // !__DRV_LED_H__