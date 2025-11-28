#include "lib_display_core.h"
#include "drv_aip6932.h" // 引用底层驱动

// 共阴极 7段数码管段码表 (0-9, A-F, 空)
static const uint8_t FONT_MAP[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, // 0-7
    0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, // 8-9, A-F
    0x00                                            // Space (index 16)
};

// 本地显存 (AIP6932 最大支持 12 Grid for 4 digit groups? 实际上 SSOP20 只能做有限的Grid)
// 假设按照 6932 手册，Grid 0-11 (具体看芯片手册，这里分配 16 字节足够)
static uint8_t g_vram[16];
static bool g_is_dirty = false;
static uint8_t g_current_bright = 4;

// 内部函数：查表获取段码
static uint8_t _get_seg_code(char c)
{
    if (c >= '0' && c <= '9')
        return FONT_MAP[c - '0'];
    if (c >= 'A' && c <= 'F')
        return FONT_MAP[c - 'A' + 10];
    if (c >= 'a' && c <= 'f')
        return FONT_MAP[c - 'a' + 10];
    return 0x00; // 默认熄灭
}

void Lib_Disp_Init(void)
{
    // 调用底层硬件初始化
    AIP6932_Init_HW();
    // 清空缓存
    for (int i = 0; i < 16; i++)
        g_vram[i] = 0x00;
    g_is_dirty = true;
}

void Lib_Disp_SetBrightness(uint8_t val)
{
    if (g_current_bright != val)
    {
        g_current_bright = val;
        g_is_dirty = true; // 亮度改变也需要重新发送控制命令
    }
}

void Lib_Disp_SetDigit(Disp_Zone_t zone, uint8_t idx, char c, bool dot)
{
    if (zone >= DISP_ZONE_MAX)
        return;

    // 1. 获取映射配置
    const Zone_Map_t *map = &ZONE_MAPPING[zone];

    // 2. 计算物理地址 (Physical Grid)
    // 假设每个 Zone 是 4 位
    uint8_t phys_grid;
    if (map->reverse_scan)
    {
        phys_grid = map->start_grid + (3 - idx);
    }
    else
    {
        phys_grid = map->start_grid + idx;
    }

    if (phys_grid >= 16)
        return; // 防止越界

    // 3. 组合段码
    uint8_t seg = _get_seg_code(c);
    if (dot)
        seg |= 0x80; // 假设最高位是 DP

    // 4. 写入缓存并标记脏
    if (g_vram[phys_grid] != seg)
    {
        g_vram[phys_grid] = seg;
        g_is_dirty = true;
    }
}

void Lib_Disp_Commit(void)
{
    // 脏检查：如果没有变化，直接退出，不占用总线
    if (!g_is_dirty)
        return;

    // 调用驱动层发送数据
    // 注意：AIP6932 支持地址自增写入，这里一次性发完效率最高
    AIP6932_Write_Buffer(g_vram, 12, g_current_bright);

    g_is_dirty = false;
}