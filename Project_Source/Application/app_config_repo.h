#ifndef __APP_CONFIG_REPO_H__
#define __APP_CONFIG_REPO_H__

#include <stdint.h>
#include "app_config_entity.h"

/* 初始化 Repository (Load from Flash) */
void ConfigRepo_Init(void);

/* 触发保存 (Persist to Flash) */
void ConfigRepo_Save(void);

/* 恢复出厂设置 */
void ConfigRepo_FactoryReset(void);

/* ========================================== */
/* Service / Getters & Setters                */
/* ========================================== */

uint16_t Service_GetTargetVoltage(void);
void Service_SetTargetVoltage(uint16_t val);

uint8_t Service_GetBatteryType(void);
void Service_SetBatteryType(uint8_t type);

/* ... 你可以继续添加剩下的 17 个 ... */
void Service_SetLedMode(uint8_t mode);
uint8_t Service_GetLedMode(void);
#endif