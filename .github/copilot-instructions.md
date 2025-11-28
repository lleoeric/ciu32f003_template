# CIU32F003 Template Project - AI Coding Guide

## Project Overview

Embedded firmware template for **CIU32F003 (Cortex-M0+)** MCU with 24KB Flash and 3KB RAM. Features a Spring Data-inspired layered architecture for Flash-based parameter persistence and LCD display management.

## Critical Hardware Constraints

### ⚠️ Flash Operations & XIP Crashes

- **RAM_FUNC Requirement**: Flash erase/write functions MUST use `__RAM_FUNC` attribute to avoid XIP (Execute-In-Place) crashes
- **Location**: All Flash-writing code in `drv_flash.c` uses `__attribute__((section(".ramfunc")))`
- **Reason**: Cannot execute from Flash while erasing/writing it
- **Never modify**: `drv_flash.c` - this is tuned for hardware timing

### M0+ Memory Alignment

- **No unaligned access**: Cortex-M0+ lacks unaligned memory access hardware
- **Struct padding**: Always pad structs to 2 or 4-byte boundaries (see `app_config_entity.h`)
- **Example**:
  ```c
  uint8_t contrast;
  uint8_t _padding[3];  // Manual padding for alignment
  ```

### Programmer Requirements

- **Must use**: CIU32 Programmer with "Run after programming" enabled
- **Avoid**: PyOCD/OpenOCD may cause SWD hangs due to incorrect reset timing

## Architecture Layers (Spring Data Pattern)

### 1. Driver Layer (DO NOT MODIFY)

- `drv_flash.c/.h` - Atomic Flash operations with RAM_FUNC
- `lib_flash_jpa.c/.h` - JPA-like persistence framework with dirty checking

### 2. Entity Layer (Define Data Structures)

- `app_config_entity.h` - Business parameter structs (add fields here)
- `app_display_entity.h` - Display state entity
- **Pattern**: Add magic header (0x5AA5) and checksum footer for validation

### 3. Repository Layer (Business Logic)

- `app_config_repo.c/.h` - CRUD operations, default values, validation
- **Pattern**: Implement `Service_Get*()` / `Service_Set*()` pairs for each field
- **Example workflow**:
  ```c
  ConfigRepo_Init();           // Load from Flash (call once at startup)
  uint8_t val = Service_GetLedMode();
  Service_SetLedMode(2);       // Modify in RAM only
  ConfigRepo_Save();           // Persist to Flash (dirty check automatic)
  ```

### 4. Service Layer (UI Logic)

- `app_display_service.c/.h` - Display control abstraction
- `lib_display_core.c/.h` - Hardware-agnostic LCD driver

## Adding New Parameters (Checklist)

1. **Define in Entity** (`app_config_entity.h`):

   ```c
   uint8_t contrast;     // New field
   uint8_t _padding[1];  // Maintain alignment
   ```

2. **Set Default** (`app_config_repo.c::ConfigRepo_FactoryReset`):

   ```c
   g_repo_instance.contrast = 50;
   ```

3. **Implement Getters/Setters** (`app_config_repo.c/.h`):

   ```c
   uint8_t Service_GetContrast(void);
   void Service_SetContrast(uint8_t val);
   ```

4. **Use in Application** (`main.c`):
   ```c
   ConfigRepo_Init();
   uint8_t c = Service_GetContrast();
   Service_SetContrast(60);
   ConfigRepo_Save();  // Only writes if data changed (dirty check)
   ```

## Key Mechanisms

### Dirty Checking

- `JPA_Save()` uses `memcmp()` to skip Flash writes if data unchanged
- Returns `JPA_SKIPPED` when no changes detected (preserves Flash lifespan)
- **Cost**: ~30ms for actual erase/write, 0ms when skipped

### Flash Storage Layout

- **Location**: Last page @ `0x00005E00` (512 bytes)
- **Capacity**: ~250 uint16_t parameters (508 bytes usable after header/checksum)
- **Structure**: `[magic_head][params...][checksum]`

## Build System

### Toolchain: ARM Compiler 6 (AC6)

- Target: Cortex-M0+ (no FPU)
- Standard: C99 for C, C++11 for C++
- Optimization: `-O0` (Debug), microLIB enabled

### Build Configuration (`.eide/eide.yml`)

- **Entry point**: `Vendor_Library/CMSIS/Device/Startup/arm/startup_ciu32f003.s`
- **Memory layout**: Flash 24KB @ 0x00000000, RAM 3KB @ 0x20000000
- **Key flags**: `--diag_suppress=L6329`, `--one_elf_section_per_function`

### Build Commands

```powershell
# EIDE project - use VS Code tasks or EIDE extension UI
# No direct CLI commands - managed by EIDE toolchain
```

## Code Style (`.clang-format`)

- **Base style**: Microsoft with Linux braces
- **Indent**: 4 spaces (no tabs)
- **Key rules**:
  - `AlignConsecutiveMacros: AcrossEmptyLines`
  - `AllowShortIfStatementsOnASingleLine: true`
  - `SortIncludes: false` (preserve manual include order)

## Common Patterns

### Naming Conventions

- **Types**: `CamelCase_t` suffix (e.g., `AppConfig_Entity_t`)
- **Functions**: `Module_Action` prefix (e.g., `ConfigRepo_Save`, `Drv_Flash_ErasePage`)
- **Services**: `Service_GetX` / `Service_SetX` pairs
- **Private functions**: Static with `_` prefix (e.g., `_CalcChecksum`)

### Includes

- Hardware headers: `#include "ciu32f003_std.h"`
- Vendor lib: `Vendor_Library/CIU32F003_Lib/Include`
- Always include in order: Standard → Vendor → Driver → Application

### Error Handling

- Use enum return codes (e.g., `DrvFlash_Status`, `JPA_Status_t`)
- Validate magic + checksum in `ConfigRepo_Init()`, auto-restore on corruption

## Documentation

- **Doxygen**: Run `doxygen Doxyfile` to generate `doc/html/`
- **Custom theme**: See `doxygen-custom/` for CSS/JS tweaks
- **Manual**: See `CIU32F003 通用 Flash 存储模块使用指南.md` for detailed Flash usage guide

## When to Ask Human

- Changing memory layout or scatter file
- Modifying Flash driver timing (erase/write cycles)
- Adding new hardware peripherals (need datasheet specs)
- Performance issues (likely M0+ alignment or Flash bottlenecks)
