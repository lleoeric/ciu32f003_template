#include "app_display_entity.h"
#include <string.h> // for memset

// 全局静态实例
static AppDisp_Entity_t g_disp_entity;

// 获取单例指针
AppDisp_Entity_t *AppDisp_GetEntity(void)
{
    return &g_disp_entity;
}

// 可选：如果需要重置数据
void AppDisp_Reset(void)
{
    memset(&g_disp_entity, 0, sizeof(AppDisp_Entity_t));
    g_disp_entity.brightness = 4; // 默认中等亮度
    g_disp_entity.is_display_on = true;
}