#ifndef __LIB_DISPLAY_CORE_H__
#define __LIB_DISPLAY_CORE_H__

#include <stdint.h>
#include <stdbool.h>
#include "lib_display_config.h"

// 初始化
void Lib_Disp_Init(void);

// 设置亮度
void Lib_Disp_SetBrightness(uint8_t val);

// 设置某一位的字符 (只写 RAM，不发通讯)
// zone: 哪个区, idx: 区内第几位(0-3), c: 字符('0'-'9'), dot: 是否显示点
void Lib_Disp_SetDigit(Disp_Zone_t zone, uint8_t idx, char c, bool dot);

// 提交刷新 (将 RAM 写入芯片，含脏检查优化)
void Lib_Disp_Commit(void);

#endif // __LIB_DISPLAY_CORE_H__