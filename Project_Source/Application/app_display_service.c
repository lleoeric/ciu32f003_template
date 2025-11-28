#include "app_display_service.h"
#include "app_display_entity.h"
#include "lib_display_config.h" // 获取区域定义
#include "lib_display_core.h"   // 调用中间层

// 辅助函数：简单的浮点转定点字符逻辑 (避免 printf)
// value: 数值, dot_pos: 小数点位置(从右向左 0-3), zone: 显示区域
static void _render_float(Disp_Zone_t zone, float value, uint8_t dot_pos)
{
    // 简单实现：假设 4 位数码管
    // 将 float 转为整形处理，例如 220.5 -> 2205
    int32_t int_val;

    // 根据小数点位置放大数值
    if (dot_pos == 0)
        int_val = (int32_t)value;
    else if (dot_pos == 1)
        int_val = (int32_t)(value * 10);
    else if (dot_pos == 2)
        int_val = (int32_t)(value * 100);
    else
        int_val = (int32_t)(value * 1000);

    // 限制范围 0-9999
    if (int_val > 9999)
        int_val = 9999;
    if (int_val < 0)
        int_val = 0;

    // 拆分每一位
    uint8_t d0 = int_val % 10;
    uint8_t d1 = (int_val / 10) % 10;
    uint8_t d2 = (int_val / 100) % 10;
    uint8_t d3 = (int_val / 1000) % 10;

    // 调用 Core 层接口显示
    // 注意：这里假设从左到右 index 是 0,1,2,3
    Lib_Disp_SetDigit(zone, 0, d3 + '0', (dot_pos == 3));
    Lib_Disp_SetDigit(zone, 1, d2 + '0', (dot_pos == 2));
    Lib_Disp_SetDigit(zone, 2, d1 + '0', (dot_pos == 1));
    Lib_Disp_SetDigit(zone, 3, d0 + '0', (dot_pos == 0));
}

// 辅助函数：显示整数
static void _render_int(Disp_Zone_t zone, uint16_t value)
{
    if (value > 9999)
        value = 9999;

    Lib_Disp_SetDigit(zone, 0, (value / 1000) % 10 + '0', false);
    Lib_Disp_SetDigit(zone, 1, (value / 100) % 10 + '0', false);
    Lib_Disp_SetDigit(zone, 2, (value / 10) % 10 + '0', false);
    Lib_Disp_SetDigit(zone, 3, (value % 10) + '0', false);
}

// --- API 实现 ---

void Service_Disp_Init(void)
{
    // 初始化 Core 层
    Lib_Disp_Init();

    // 加载默认值
    AppDisp_Entity_t *entity = AppDisp_GetEntity();
    entity->brightness = 4;
    entity->is_display_on = true;

    // 强制刷新一次配置
    Lib_Disp_SetBrightness(entity->brightness);
}

void Service_Disp_SetVoltage(float vol)
{
    AppDisp_Entity_t *entity = AppDisp_GetEntity();
    if (entity->voltage != vol)
    {
        entity->voltage = vol;
        // 电压通常保留1位小数，如 220.5
        _render_float(DISP_ZONE_VOLT, vol, 1);
    }
}

void Service_Disp_SetCurrent(float curr)
{
    AppDisp_Entity_t *entity = AppDisp_GetEntity();
    if (entity->current != curr)
    {
        entity->current = curr;
        // 电流通常保留2位小数，如 1.50
        _render_float(DISP_ZONE_CURR, curr, 2);
    }
}

void Service_Disp_SetPower(uint16_t watt)
{
    AppDisp_Entity_t *entity = AppDisp_GetEntity();
    if (entity->power != watt)
    {
        entity->power = watt;
        // 功率显示整数
        _render_int(DISP_ZONE_POWER, watt);
    }
}

void Service_Disp_SetBrightness(uint8_t level)
{
    if (level > 7)
        level = 7;
    AppDisp_GetEntity()->brightness = level;
    Lib_Disp_SetBrightness(level);
}

void Service_Disp_Task(void)
{
    AppDisp_Entity_t *entity = AppDisp_GetEntity();
    entity->tick_counter++;

    // 可以在这里处理闪烁逻辑 (例如每 500ms 关掉显示)
    // if (entity->is_error_mode && (entity->tick_counter % 50 == 0)) { ... }

    // 最终提交显存到硬件 (执行脏检查)
    Lib_Disp_Commit();
}