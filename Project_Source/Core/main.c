#include "main.h"            // 包含硬件定义
#include "app_config_repo.h" // 【核心】引用我们的业务仓库
#include "drv_clock.h"       // 时钟驱动
#include "drv_led.h"         // LED 驱动

int main(void)
{
    /* 1. 基础硬件初始化 */
    system_clock_config();
    std_delay_init();
    drv_led_init();

    /* 2. 【仓库初始化】 */
    /* 这一步会从 Flash 加载数据到 RAM，如果 Flash 是空的，会自动恢复出厂设置 */
    ConfigRepo_Init();

    /* 3. 读取当前的 LED 模式 */
    uint8_t current_mode = Service_GetLedMode();

    /* 4. 准备下一次启动的模式 (状态机逻辑) */
    /* 0 -> 1 -> 2 -> 0 ... */
    uint8_t next_mode = current_mode + 1;
    if (next_mode > 2)
    {
        next_mode = 0;
    }

    /* 5. 更新 RAM 中的参数 */
    Service_SetLedMode(next_mode);

    /* 6. 【保存到 Flash】 */
    /* 这里演示“改了就存”。底层会自动做脏检查，
       但因为我们改了 next_mode，数据肯定变了，所以会触发真实的 Flash 写入 */
    ConfigRepo_Save();

    /* 7. 执行当前模式的动作 (死循环) */
    /* 注意：此时 Flash 已经写完了，我们可以安心跑业务逻辑 */

    if (current_mode == 0)
    {
        // 模式 0: 灭
        drv_led_on();
        while (1)
            ; // 挂起
    }
    else if (current_mode == 1)
    {
        // 模式 1: 常亮
        drv_led_off();
        while (1)
            ; // 挂起
    }
    else
    {
        // 模式 2 (或其他): 闪烁
        while (1)
        {
            drv_led_toggle();
            std_delayms(1000);
        }
    }
}