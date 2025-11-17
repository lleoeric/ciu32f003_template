#ifndef __LIB_FLASH_JPA_H__
#define __LIB_FLASH_JPA_H__

#include <stdint.h>
#include <stdbool.h>

/* 状态码 */
typedef enum {
    JPA_OK = 0,
    JPA_SKIPPED,   // 数据未变，跳过写入（脏检查生效）
    JPA_ERR_ERASE, // 擦除失败
    JPA_ERR_WRITE, // 写入失败
    JPA_ERR_VERIFY // 校验失败
} JPA_Status_t;

/**
 * @brief  通用加载 (Load)
 * @param  flash_addr: Flash 物理地址
 * @param  p_entity:   RAM 中的实体指针 (void*)
 * @param  size:       实体大小 (sizeof)
 */
void JPA_Load(uint32_t flash_addr, void *p_entity, uint32_t size);

/**
 * @brief  通用保存 (Save) - 带脏检查
 * @param  flash_addr: Flash 物理地址 (必须是页起始地址)
 * @param  p_entity:   RAM 中的实体指针 (void*)
 * @param  size:       实体大小
 * @return JPA_Status_t
 */
JPA_Status_t JPA_Save(uint32_t flash_addr, const void *p_entity, uint32_t size);

#endif