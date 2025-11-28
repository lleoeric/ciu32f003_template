#ifndef __APP_DISPLAY_SERVICE_H__
#define __APP_DISPLAY_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>

// 这是一个纯业务层，不包含任何硬件头文件

// 初始化显示服务
void Service_Disp_Init(void);

// 核心业务 API
void Service_Disp_SetVoltage(float vol);
void Service_Disp_SetCurrent(float curr);
void Service_Disp_SetPower(uint16_t watt);

// 状态控制 API
void Service_Disp_SetBrightness(uint8_t level);
void Service_Disp_SetPowerState(bool on);

// 周期性任务 (建议 10ms 或 20ms 调用一次)
// 负责处理闪烁、脏数据刷新
void Service_Disp_Task(void);

#endif // __APP_DISPLAY_SERVICE_H__