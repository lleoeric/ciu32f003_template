#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

#include "ciu32f003_std.h" // 确保包含官方标准库头文件

/* ================= 类型定义 ================= */

/* Flash 操作状态码 */
typedef enum {
    DRV_FLASH_OK = 0,
    DRV_FLASH_ERR_BUSY,
    DRV_FLASH_ERR_ERASE,
    DRV_FLASH_ERR_PROGRAM,
    DRV_FLASH_ERR_VERIFY
} DrvFlash_Status;

/* ================= 函数声明 ================= */

/**
 * @brief 擦除指定页
 * @param page_addr 页起始地址
 */
DrvFlash_Status Drv_Flash_ErasePage(uint32_t page_addr);

/**
 * @brief 写入一个字 (32-bit)
 * @param addr 写入地址 (必须4字节对齐)
 * @param data 写入的数据
 */
DrvFlash_Status Drv_Flash_WriteWord(uint32_t addr, uint32_t data);

#endif