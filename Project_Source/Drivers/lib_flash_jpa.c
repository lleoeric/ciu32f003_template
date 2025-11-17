#include "lib_flash_jpa.h"
#include "ciu32f003_std_flash.h"
#include "drv_flash.h" // 依赖我们之前写的底层驱动
#include <string.h>    // memcpy, memcmp

/* 内部工具：向上对齐到4字节 */
static inline uint32_t _align_up_4(uint32_t val)
{
    return (val + 3) & ~0x03;
}

void JPA_Load(uint32_t flash_addr, void *p_entity, uint32_t size)
{
    // M0+ 必须使用 memcpy 避免非对齐访问导致的 HardFault
    memcpy(p_entity, (void *)flash_addr, size);
}

JPA_Status_t JPA_Save(uint32_t flash_addr, const void *p_entity, uint32_t size)
{
    // 1. 【脏检查】(Dirty Check)
    // 如果 Flash 里的数据和 RAM 里的完全一样，直接返回，不损耗寿命
    if (memcmp((void *)flash_addr, p_entity, size) == 0)
    {
        return JPA_SKIPPED;
    }

    // 2. 擦除页面 (Transaction Begin)
    // 注意：这里假设你的数据存储在独立的一页中
    if (Drv_Flash_ErasePage(flash_addr) != DRV_FLASH_OK)
    {
        return JPA_ERR_ERASE;
    }

    // 3. 循环写入 (Persist)
    uint32_t current_addr = flash_addr;
    uint32_t words_to_write = _align_up_4(size) / 4; // 转换成 Word 数量

    // 使用临时变量处理非对齐数据，保证 Flash 写入始终是 32位 对齐的
    uint32_t temp_word;
    const uint8_t *p_byte_src = (const uint8_t *)p_entity;

    for (uint32_t i = 0; i < words_to_write; i++)
    {
        // 组装 4 字节，即使源数据大小不是4的倍数也是安全的
        // 这里虽然稍微慢一点点，但比直接强转指针更安全
        temp_word = 0;
        for (int j = 0; j < 4; j++)
        {
            uint32_t byte_idx = i * 4 + j;
            if (byte_idx < size)
            {
                temp_word |= (uint32_t)(p_byte_src[byte_idx]) << (j * 8);
            }
            else
            {
                temp_word |= 0xFFFFFFFF << (j * 8); // 填充 0xFF
            }
        }

        // 写入 Flash
        if (Drv_Flash_WriteWord(current_addr, temp_word) != DRV_FLASH_OK)
        {
            return JPA_ERR_WRITE;
        }

        current_addr += 4;
    }

    // 4. 校验 (Verify)
    if (memcmp((void *)flash_addr, p_entity, size) != 0)
    {
        return JPA_ERR_VERIFY;
    }

    return JPA_OK;
}