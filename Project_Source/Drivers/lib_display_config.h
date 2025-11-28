#ifndef __LIB_DISPLAY_CONFIG_H__
#define __LIB_DISPLAY_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>

// 定义逻辑显示分区 (根据你的需求：电压、电流、功率)
typedef enum {
    DISP_ZONE_VOLT  = 0, // 0号区
    DISP_ZONE_CURR  = 1, // 1号区
    DISP_ZONE_POWER = 2, // 2号区
    DISP_ZONE_MAX
} Disp_Zone_t;

// 区域映射配置结构体
typedef struct {
    uint8_t start_grid; // 该区域在芯片上的起始 Grid (0-11)
    bool reverse_scan;  // 是否反向 (例如 PCB 走线反了)
} Zone_Map_t;

// 导出配置表
extern const Zone_Map_t ZONE_MAPPING[DISP_ZONE_MAX];

#endif // __LIB_DISPLAY_CONFIG_H__