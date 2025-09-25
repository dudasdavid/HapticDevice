#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// DRV2605L 7-bit I2C address
#define DRV2605L_ADDR        (0x5A << 1)  // HAL expects 8-bit address (left-shifted)

// Registers (subset)
#define DRV2605_REG_STATUS       0x00
#define DRV2605_REG_MODE         0x01
#define DRV2605_REG_RTPIN        0x02
#define DRV2605_REG_LIBRARY      0x03
#define DRV2605_REG_WAVESEQ1     0x04   // … up to 0x0B (8 slots)
#define DRV2605_REG_WAVESEQ(n)   (DRV2605_REG_WAVESEQ1 + ((n) & 0x07))
#define DRV2605_REG_GO           0x0C
#define DRV2605_REG_OVERDRIVE    0x0D
#define DRV2605_REG_SUSTAINPOS   0x0E
#define DRV2605_REG_SUSTAINNEG   0x0F
#define DRV2605_REG_BREAK        0x10
#define DRV2605_REG_AUDIOMAX     0x12
#define DRV2605_REG_FEEDBACK     0x1A
#define DRV2605_REG_CONTROL1     0x1B
#define DRV2605_REG_CONTROL2     0x1C
#define DRV2605_REG_CONTROL3     0x1D

// MODE values (most common)
#define DRV2605_MODE_INTTRIG     0x00  // internal trigger (waveform sequencer)
#define DRV2605_MODE_EXTEDGE     0x01
#define DRV2605_MODE_EXTLEVEL    0x02
#define DRV2605_MODE_PWMANALOG   0x03
#define DRV2605_MODE_AUDIOVIBE   0x04
#define DRV2605_MODE_RTP         0x05
#define DRV2605_MODE_DIAG        0x06
#define DRV2605_MODE_AUTOCAL     0x07

// Library (example values; 1 is a solid default for ERM)
#define DRV2605_LIB_EMPTY        0x00
#define DRV2605_LIB_ERM          0x01   // basic ERM library

// Helpers
HAL_StatusTypeDef DRV2605_Write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);
HAL_StatusTypeDef DRV2605_Read8 (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val);

// Public API
HAL_StatusTypeDef DRV2605_Init_ERM(I2C_HandleTypeDef *hi2c, uint8_t library_sel);
HAL_StatusTypeDef DRV2605_SetEffectSlot(I2C_HandleTypeDef *hi2c, uint8_t slot, uint8_t effect);
HAL_StatusTypeDef DRV2605_LoadPattern(I2C_HandleTypeDef *hi2c, const uint8_t *effects, uint8_t count);
HAL_StatusTypeDef DRV2605_Play(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef DRV2605_Stop(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif
