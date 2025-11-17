#include "drv_flash.h"

/* ========================================== */
/* 1. 定义 RAM 函数宏 (根据编译器)             */
/* ========================================== */
#if defined(__CC_ARM) || defined(__GNUC__)
#define __RAM_FUNC __attribute__((section(".ramfunc")))
#elif defined(__ICCARM__)
#define __RAM_FUNC __ramfunc
#endif

/* ========================================== */
/* 2. 私有 RAM 函数：执行具体的硬件操作         */
/* (完全内联，不调用任何外部函数)            */
/* ========================================== */

/**
 * @brief 在 RAM 中执行页擦除
 */
__RAM_FUNC
static DrvFlash_Status _ISP_ErasePage(uint32_t addr)
{
    /* 1. 解锁 Flash */
    if ((FLASH->CR & FLASH_CR_LOCK) == FLASH_CR_LOCK)
    {
        FLASH->CRKEY = FLASH_CR_KEY1;
        FLASH->CRKEY = FLASH_CR_KEY2;
    }

    /* 2. 清除标志位 */
    FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);

    /* 3. 设置擦除模式 */
    MODIFY_REG(FLASH->CR, FLASH_CR_OP_MODE, FLASH_MODE_PAGE_ERASE);

    /* 4. 触发擦除 (写入任意数据) */
    *(uint32_t *)addr = 0xFFFFFFFF;

    /* 5. 等待忙 (BSY) */
    while ((FLASH->SR & FLASH_FLAG_BSY))
        ;

    /* 6. 检查写保护错误 */
    if ((FLASH->SR & FLASH_FLAG_WRPERR) != 0x00000000U)
    {
        FLASH->CR |= FLASH_CR_LOCK; // 立即加锁
        return DRV_FLASH_ERR_ERASE;
    }

    /* 7. 清理标志并复位模式 */
    FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
    MODIFY_REG(FLASH->CR, FLASH_CR_OP_MODE, FLASH_MODE_IDLE);

    /* 8. 锁定 */
    FLASH->CR |= FLASH_CR_LOCK;

    return DRV_FLASH_OK;
}

/**
 * @brief 在 RAM 中执行字编程 (32-bit)
 */
__RAM_FUNC
static DrvFlash_Status _ISP_ProgramWord(uint32_t addr, uint32_t data)
{
    /* 1. 解锁 */
    if ((FLASH->CR & FLASH_CR_LOCK) == FLASH_CR_LOCK)
    {
        FLASH->CRKEY = FLASH_CR_KEY1;
        FLASH->CRKEY = FLASH_CR_KEY2;
    }

    /* 2. 清标志 */
    FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);

    /* 3. 设置编程模式 */
    MODIFY_REG(FLASH->CR, FLASH_CR_OP_MODE, FLASH_MODE_PROGRAM);

    /* 4. 写入数据 */
    *(uint32_t *)addr = data;

    /* 5. 等待忙 */
    while ((FLASH->SR & FLASH_FLAG_BSY))
        ;

    /* 6. 检查错误 */
    if ((FLASH->SR & FLASH_FLAG_WRPERR) != 0x00000000U)
    {
        FLASH->CR |= FLASH_CR_LOCK;
        return DRV_FLASH_ERR_PROGRAM;
    }

    /* 7. 硬件回读校验 (可选，但在 RAM 里做很方便) */
    if (*((volatile uint32_t *)addr) != data)
    {
        FLASH->CR |= FLASH_CR_LOCK;
        return DRV_FLASH_ERR_VERIFY;
    }

    /* 8. 清理并锁定 */
    FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
    MODIFY_REG(FLASH->CR, FLASH_CR_OP_MODE, FLASH_MODE_IDLE);
    FLASH->CR |= FLASH_CR_LOCK;

    return DRV_FLASH_OK;
}

/* ========================================== */
/* 3. 公共 API：给上层 (JPA) 调用的接口        */
/* ========================================== */

DrvFlash_Status Drv_Flash_ErasePage(uint32_t page_addr)
{
    return _ISP_ErasePage(page_addr);
}

DrvFlash_Status Drv_Flash_WriteWord(uint32_t addr, uint32_t data)
{
    // 简单的对齐检查
    if (addr % 4 != 0)
    {
        return DRV_FLASH_ERR_PROGRAM;
    }
    return _ISP_ProgramWord(addr, data);
}