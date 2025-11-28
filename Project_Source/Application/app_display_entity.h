#ifndef __APP_DISPLAY_ENTITY_H__
#define __APP_DISPLAY_ENTITY_H__

#include <stdint.h>
#include <stdbool.h>

/* * 业务数据实体定义
 * 这里定义了你想在屏幕上显示的所有“状态”
 */
typedef struct {
    // --- 核心业务数值 ---
    float voltage;  // 电压值 (e.g., 220.5)
    float current;  // 电流值 (e.g., 1.500)
    uint16_t power; // 功率值 (e.g., 3300)

    // --- 界面状态控制 ---
    uint8_t brightness; // 亮度等级 (0-7)
    bool is_display_on; // 屏幕总开关
    bool is_error_mode; // 是否处于报警闪烁模式

    // --- 内部计数器 (用于闪烁逻辑) ---
    uint32_t tick_counter;

} AppDisp_Entity_t;

// 全局访问接口 (单例模式)
AppDisp_Entity_t *AppDisp_GetEntity(void);

#endif // __APP_DISPLAY_ENTITY_H__