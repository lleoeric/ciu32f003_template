#include "drv_aip6932.h"

// --- 寄存器操作宏 (追求 -O3 极致性能) ---
// CIU32F003 的 BSRR (Bit Set/Reset) 和 IDR (Input Data)
// 请根据具体手册确认寄存器名称，通常是 BSRR 或 BSRRL/BSRRH

#define STB_LOW() (AIP_STB_PORT->BRR = AIP_STB_PIN)   // Reset
#define STB_HIGH() (AIP_STB_PORT->BSRR = AIP_STB_PIN) // Set

#define CLK_LOW() (AIP_CLK_PORT->BRR = AIP_CLK_PIN)
#define CLK_HIGH() (AIP_CLK_PORT->BSRR = AIP_CLK_PIN)

#define DIO_LOW() (AIP_DIO_PORT->BRR = AIP_DIO_PIN)
#define DIO_HIGH() (AIP_DIO_PORT->BSRR = AIP_DIO_PIN)
#define DIO_READ() ((AIP_DIO_PORT->IDR & AIP_DIO_PIN) ? 1 : 0)

// 简单的微秒延时 (适配 48MHz/24MHz 主频，避免被优化掉)
static void _delay_us(volatile uint32_t us)
{
    us *= 10; // 粗略校准
    while (us--)
        ;
}

// 设置 DIO 为输入模式 (用于读按键)
static void _dio_mode_input(void)
{
    // 假设 MODER 寄存器，2位控制一个 pin
    // 需要根据 pin index 计算位移，这里简化伪代码：
    // MODIFY_REG(AIP_DIO_PORT->MODER, 0x03 << (5*2), 0x00 << (5*2)); // Input
}

// 设置 DIO 为输出模式
static void _dio_mode_output(void)
{
    // MODIFY_REG(AIP_DIO_PORT->MODER, 0x03 << (5*2), 0x01 << (5*2)); // Output
}

// 发送一个字节 (LSB First)
static void _send_byte(uint8_t dat)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        CLK_LOW();
        // 准备数据
        if (dat & 0x01)
            DIO_HIGH();
        else
            DIO_LOW();
        _delay_us(1);

        CLK_HIGH(); // 上升沿读入
        _delay_us(1);
        dat >>= 1;
    }
}

// --- 公开函数 ---

void AIP6932_Init_HW(void)
{
    // 1. 开启 GPIO 时钟 (RCC)
    // 根据 CIU32 手册开启 Port A 和 Port B 时钟
    // RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB;

    // 2. 配置引脚为推挽输出
    // GPIO_Init(...)

    // 3. 默认拉高
    STB_HIGH();
    CLK_HIGH();
    DIO_HIGH();
}

void AIP6932_Write_Buffer(uint8_t *data, uint8_t len, uint8_t brightness)
{
    // Command 1: 设置数据命令 (普通模式, 地址自增)
    STB_LOW();
    _send_byte(0x40);
    STB_HIGH();
    _delay_us(1);

    // Command 2: 设置地址命令 (起始地址 0xC0)
    STB_LOW();
    _send_byte(0xC0);

    // 连续写入数据
    for (uint8_t i = 0; i < len; i++)
    {
        _send_byte(data[i]);
    }
    STB_HIGH();
    _delay_us(1);

    // Command 3: 显示控制命令 (亮度设置 + 开启显示)
    // 0x88 | (brightness & 0x07) -> 开启
    // 0x80 -> 关闭
    STB_LOW();
    _send_byte(0x88 | (brightness & 0x07));
    STB_HIGH();
}

uint8_t AIP6932_Read_Keys(void)
{
    uint8_t key_data[4] = {0};

    STB_LOW();
    _send_byte(0x42); // 读按键命令

    // 切换 DIO 方向
    _dio_mode_input();
    _delay_us(2); // 等待抗干扰

    // 读 4 个字节 (具体看手册，6932通常返回4字节按键数据)
    for (int i = 0; i < 4; i++)
    {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; bit++)
        {
            CLK_LOW();
            _delay_us(1);
            CLK_HIGH(); // 上升沿芯片输出数据
            if (DIO_READ())
                byte |= (1 << bit); // LSB first
            _delay_us(1);
        }
        key_data[i] = byte;
    }

    STB_HIGH();
    _dio_mode_output(); // 恢复输出

    // 简单处理：返回第一个字节 (根据实际按键矩阵解析)
    return key_data[0];
}