#ifndef __APP_CONFIG_ENTITY_H__
#define __APP_CONFIG_ENTITY_H__

#include <stdint.h>

/* ========================================== */
/* Entity Definition (纯数据结构)             */
/* ========================================== */

typedef struct {
    // --- Header ---
    uint16_t magic_head; // 0x5AA5

    //------------------TEST-------------------------
    // --- 业务参数 ---
    // 我们添加一个用于测试 LED 模式的参数
    uint8_t led_work_mode; // 0=OFF, 1=ON, 2=BLINK

    uint8_t padding[3]; // 手动补齐到4字节 (M0+ 最佳实践)

    //------------------TEST-------------------------
    // --- 19 Parameters ---
    uint16_t target_voltage;     // 0x81
    uint16_t target_frequency;   // 0x82
    uint16_t charging_priority;  // 0x90
    uint16_t max_grid_charge_a;  // 0x91
    uint16_t max_total_charge_a; // 0x92
    uint16_t batt_ret_grid_v;    // 0x93
    uint16_t batt_ret_batt_v;    // 0x94

    uint8_t battery_type;     // 0x95
    uint8_t reserved_padding; // 【手动补齐】为了美观和对齐

    uint16_t batt_low_alarm_v;      // 0x96
    uint16_t batt_shutdown_v;       // 0x97
    uint16_t batt_cv_v;             // 0x98
    uint16_t batt_float_v;          // 0x99
    uint16_t batt_low_soc_shutdown; // 0x9A
    uint16_t batt_low_soc_to_grid;  // 0x9B
    uint16_t batt_max_discharge_a;  // 0x9C
    uint16_t grid_low_volt_cut;     // 0xA0
    uint16_t grid_high_volt_cut;    // 0xA1

    uint8_t alarm_input_loss;  // 0xB0
    uint8_t alarm_buzzer_mute; // 0xB1

    // --- Footer ---
    uint16_t checksum; // 校验和
} AppConfig_Entity_t;

#endif