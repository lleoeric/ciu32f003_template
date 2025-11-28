#ifndef __DRV_AIP6932_H__
#define __DRV_AIP6932_H__

#include <stdint.h>
#include "ciu32f003.h" // 替换为你实际的芯片头文件

// 硬件引脚定义 (用户在此修改)
// 假设：
// STB -> GPIOA_1
// CLK -> GPIOA_2
// DIO -> GPIOB_5

#define AIP_STB_PORT GPIOB
#define AIP_STB_PIN  GPIO_PIN_1

#define AIP_CLK_PORT GPIOB
#define AIP_CLK_PIN  GPIO_PIN_2

#define AIP_DIO_PORT GPIOB
#define AIP_DIO_PIN  GPIO_PIN_5

// 初始化 GPIO
void AIP6932_Init_HW(void);

// 核心发送函数
// data: 显存数据指针, len: 数据长度, brightness: 亮度控制字
void AIP6932_Write_Buffer(uint8_t *data, uint8_t len, uint8_t brightness);

// 读按键 (如果需要)
uint8_t AIP6932_Read_Keys(void);

#endif // __DRV_AIP6932_H__