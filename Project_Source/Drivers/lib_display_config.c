#include "lib_display_config.h"

// ！！！这里是唯一需要根据 PCB 走线修改的地方 ！！！
// 假设你用了 AIP6932 的 Grid 0-11
// Zone 0 (电压): 占用 Grid 0,1,2,3
// Zone 1 (电流): 占用 Grid 4,5,6,7
// Zone 2 (功率): 占用 Grid 8,9,10,11

const Zone_Map_t ZONE_MAPPING[DISP_ZONE_MAX] = {
    [DISP_ZONE_VOLT] = {.start_grid = 0, .reverse_scan = false},
    [DISP_ZONE_CURR] = {.start_grid = 4, .reverse_scan = false},
    [DISP_ZONE_POWER] = {.start_grid = 8, .reverse_scan = false}};