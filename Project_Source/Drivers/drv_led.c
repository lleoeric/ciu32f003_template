#include "drv_led.h"

void drv_led_init(void)
{
    std_gpio_init_t gpio_config = {0};

    /* 使能LED对应的GPIO时钟 */
    std_rcc_gpio_clk_enable(LED1_CLOCK);

    /* 配置LED的IO */
    gpio_config.pin = LED1_PIN;
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pull = GPIO_PULLUP;
    gpio_config.output_type = GPIO_OUTPUT_PUSHPULL;

    /* 初始化GPIO */
    std_gpio_init(LED1_PORT, &gpio_config);
}
void drv_led_on(void)
{
    std_gpio_reset_pin(LED1_PORT, LED1_PIN);
}
void drv_led_off(void)
{
    std_gpio_set_pin(LED1_PORT, LED1_PIN);
}
void drv_led_toggle(void)
{
    std_gpio_toggle_pin(LED1_PORT, LED1_PIN);
}