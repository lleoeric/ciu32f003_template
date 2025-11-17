#include "app_config_repo.h"
#include "lib_flash_jpa.h" // 引用通用框架
#include <string.h>        // memset

/* 配置区：Flash 地址 */
#define CONFIG_FLASH_ADDR (0x00005E00) // 倒数第一页
#define CONFIG_MAGIC (0x5AA5)

/* 单例：RAM 中的影子副本 */
static AppConfig_Entity_t g_repo_instance;

/* 内部函数：计算简单的校验和 */
static uint16_t _CalcChecksum(AppConfig_Entity_t *p)
{
    uint8_t *ptr = (uint8_t *)p;
    uint16_t sum = 0;
    // 计算除去最后 checksum 字段之外的所有字节
    for (int i = 0; i < sizeof(AppConfig_Entity_t) - 2; i++)
    {
        sum += ptr[i];
    }
    return sum;
}

/* 恢复出厂默认值 */
void ConfigRepo_FactoryReset(void)
{
    memset(&g_repo_instance, 0, sizeof(AppConfig_Entity_t));

    g_repo_instance.magic_head = CONFIG_MAGIC;

    // --- 默认值设定 ---
    g_repo_instance.target_voltage = 220;
    g_repo_instance.target_frequency = 50;
    g_repo_instance.battery_type = 0; // 铅酸
    // ... 其他默认值 ...

    ConfigRepo_Save(); // 立即写入
}

void ConfigRepo_Init(void)
{
    // 1. Load
    JPA_Load(CONFIG_FLASH_ADDR, &g_repo_instance, sizeof(AppConfig_Entity_t));

    // 2. Validate Magic
    if (g_repo_instance.magic_head != CONFIG_MAGIC)
    {
        ConfigRepo_FactoryReset();
        return;
    }

    // 3. Validate Checksum
    if (_CalcChecksum(&g_repo_instance) != g_repo_instance.checksum)
    {
        ConfigRepo_FactoryReset();
        return;
    }
}

void ConfigRepo_Save(void)
{
    // 1. Update Checksum
    g_repo_instance.magic_head = CONFIG_MAGIC;
    g_repo_instance.checksum = _CalcChecksum(&g_repo_instance);

    // 2. Call JPA to Save (它会自动做脏检查)
    JPA_Status_t res = JPA_Save(CONFIG_FLASH_ADDR, &g_repo_instance, sizeof(AppConfig_Entity_t));

    if (res == JPA_ERR_WRITE || res == JPA_ERR_ERASE)
    {
        // 可以在这里加一个全局错误标志，让屏幕显示 Error
    }
}

/* ========================================== */
/* Getters & Setters Implementation           */
/* ========================================== */

uint16_t Service_GetTargetVoltage(void)
{
    return g_repo_instance.target_voltage;
}

void Service_SetTargetVoltage(uint16_t val)
{
    // 业务校验
    if (val > 500)
        return;

    g_repo_instance.target_voltage = val;

    // 注意：这里只修改 RAM。
    // 如果你想“改了就存”，可以在这里调用 ConfigRepo_Save();
    // 如果你想“退出菜单再存”，这里就不用调 Save。
}

uint8_t Service_GetBatteryType(void)
{
    return g_repo_instance.battery_type;
}

void Service_SetBatteryType(uint8_t type)
{
    if (type > 5)
        return;
    g_repo_instance.battery_type = type;
}

/* ========================================== */
/* 新增：LED 模式的 Getters & Setters          */
/* ========================================== */

/**
 * @brief 获取当前 LED 工作模式
 */
uint8_t Service_GetLedMode(void)
{
    // 直接从 Shadow RAM 读取，速度极快
    return g_repo_instance.led_work_mode;
}

/**
 * @brief 设置 LED 工作模式
 * @param mode: 0-2
 */
void Service_SetLedMode(uint8_t mode)
{
    // 1. 业务校验：防止越界
    if (mode > 2)
    {
        mode = 0; // 如果非法，重置为 0
    }

    // 2. 更新 Shadow RAM
    g_repo_instance.led_work_mode = mode;

    // 注意：这里只改了内存，没有写 Flash。
    // 决定何时写 Flash 是上层应用策略的事情。
}
