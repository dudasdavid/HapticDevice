/*
 * DRV2605L.c
 *
 *  Created on: Sep 23, 2025
 *      Author: david.dudas
 */


#include "drv2605l.h"

static HAL_StatusTypeDef write_bits(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t mask, uint8_t valueMasked)
{
    uint8_t v;
    if (DRV2605_Read8(hi2c, reg, &v) != HAL_OK) return HAL_ERROR;
    v = (v & ~mask) | (valueMasked & mask);
    return DRV2605_Write8(hi2c, reg, v);
}

HAL_StatusTypeDef DRV2605_Write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, DRV2605L_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef DRV2605_Read8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(hi2c, DRV2605L_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef DRV2605_Init_ERM(I2C_HandleTypeDef *hi2c, uint8_t library_sel)
{
    HAL_StatusTypeDef st;

    // 1) Take device out of standby + select internal trigger mode
    st = DRV2605_Write8(hi2c, DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
    if (st != HAL_OK) return st;

    // 2) Disable RTP, set some safe defaults for sequencer-related control regs
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_RTPIN, 0x00)) != HAL_OK) return st;
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_OVERDRIVE, 0x00)) != HAL_OK) return st;
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_SUSTAINPOS, 0x00)) != HAL_OK) return st;
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_SUSTAINNEG, 0x00)) != HAL_OK) return st;
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_BREAK,      0x00)) != HAL_OK) return st;
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_AUDIOMAX,   0x64)) != HAL_OK) return st; // default from common examples

    // 3) Make sure device is configured for ERM (not LRA)
    // FEEDBACK register bit7 = 0 -> ERM; 1 -> LRA
    if ((st = write_bits(hi2c, DRV2605_REG_FEEDBACK, 0x80, 0x00)) != HAL_OK) return st;

    // 4) Optional: Control3 adjustments often used with ERM (clear LRA-related settings)
    // Here we don't force any special bits; keep silicon defaults.

    // 5) Select an ERM library (1 is a good general-purpose choice)
    st = DRV2605_Write8(hi2c, DRV2605_REG_LIBRARY, library_sel);
    if (st != HAL_OK) return st;

    // 6) Clear any existing sequence (set first slot to 0 to mark end)
    if ((st = DRV2605_Write8(hi2c, DRV2605_REG_WAVESEQ1, 0x00)) != HAL_OK) return st;

    return HAL_OK;
}

HAL_StatusTypeDef DRV2605_SetEffectSlot(I2C_HandleTypeDef *hi2c, uint8_t slot, uint8_t effect)
{
    if (slot > 7) slot = 7; // 8 slots: 0..7
    return DRV2605_Write8(hi2c, DRV2605_REG_WAVESEQ(slot), effect);
}

/**
 * Load a pattern of 'count' effects into the 8-slot sequencer.
 * The driver requires a terminating 0x00 after the last effect,
 * so this function writes 0x00 after your list (or fills the rest with 0).
 */
HAL_StatusTypeDef DRV2605_LoadPattern(I2C_HandleTypeDef *hi2c, const uint8_t *effects, uint8_t count)
{
    HAL_StatusTypeDef st;

    if (count > 8) count = 8;

    // Write effects
    for (uint8_t i = 0; i < count; i++) {
        st = DRV2605_Write8(hi2c, DRV2605_REG_WAVESEQ(i), effects[i]);
        if (st != HAL_OK) return st;
    }
    // Terminate sequence with 0, and clear remaining slots
    if (count < 8) {
        st = DRV2605_Write8(hi2c, DRV2605_REG_WAVESEQ(count), 0x00);
        if (st != HAL_OK) return st;
        for (uint8_t i = count + 1; i < 8; i++) {
            st = DRV2605_Write8(hi2c, DRV2605_REG_WAVESEQ(i), 0x00);
            if (st != HAL_OK) return st;
        }
    } else {
        // If exactly 8 effects, overwrite last with 0x00 as terminator (common practice)
        st = DRV2605_Write8(hi2c, DRV2605_REG_WAVESEQ(7), 0x00);
        if (st != HAL_OK) return st;
    }

    return HAL_OK;
}

HAL_StatusTypeDef DRV2605_Play(I2C_HandleTypeDef *hi2c)
{
    // Set GO bit to 1
    return DRV2605_Write8(hi2c, DRV2605_REG_GO, 0x01);
}

HAL_StatusTypeDef DRV2605_Stop(I2C_HandleTypeDef *hi2c)
{
    // Clear GO bit to 0
    return DRV2605_Write8(hi2c, DRV2605_REG_GO, 0x00);
}
