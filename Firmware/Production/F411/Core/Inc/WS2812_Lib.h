/**
 * WS2812 Neopixel LED driver
 *
 * @Author: Nicolas Dammin, 2016
 * @Author: Dávid Dudás, 2018
 * @Author: Dániel Buga, 2018
 */

#pragma once

#include <stdint.h>

#define WS2812_NUM_LEDS_CH1     16
#define WS2812_NUM_RESET_BITS   48

#define WS2812_OUT_BITS   (WS2812_NUM_LEDS_CH1 * 24 + WS2812_NUM_RESET_BITS)

//#define WS2812_USE_TIMER
#define WS2812_USE_SPI

#define WS2812_TIM_PRESCALE     0    // F_T3  = 80 MHz (12.5ns)
#define WS2812_TIM_PERIODE      119  // F_PWM = 800 kHz (1.25us)

#define WS2812_LO_TIME          28   // 34 * 12.5ns = 0.43us
#define WS2812_HI_TIME          56   // 65 * 12.5ns = 0.81us

//--------------------------------------------------------------
// RGB LED Farbdefinition (3 x 8bit)
//--------------------------------------------------------------
typedef struct
{
    uint8_t red;    // 0...255 (als PWM-Wert)
    uint8_t green;  // 0...255 (als PWM-Wert)
    uint8_t blue;   // 0...255 (als PWM-Wert)
} WS2812_RGB_t;

//--------------------------------------------------------------
// HSV LED Farbdefinition
//--------------------------------------------------------------
typedef struct
{
    uint16_t h;     // 0...359 (in Grad, 0=R, 120=G, 240=B)
    uint8_t s;      // 0...100 (in Prozent)
    uint8_t v;      // 0...100 (in Prozent)
} WS2812_HSV_t;

//Library Interface
void WS2812_Refresh(void);
void WS2812_Clear(void);
void WS2812_HSV2RGB(WS2812_HSV_t hsv_col, WS2812_RGB_t *rgb_col);
void WS2812_One_RGB(uint32_t nr, WS2812_RGB_t rgb_col, uint8_t refresh);
void WS2812_All_RGB(WS2812_RGB_t rgb_col, uint8_t refresh);
void WS2812_One_HSV(uint32_t nr, WS2812_HSV_t hsv_col, uint8_t refresh);
void WS2812_All_HSV(WS2812_HSV_t hsv_col, uint8_t refresh);
void WS2812_ColorWheel_HSV(uint16_t phase, uint8_t refresh);
void WS2812_Shift_Left(uint8_t refresh);
void WS2812_Shift_Right(uint8_t refresh);
void WS2812_Rotate_Left(uint8_t refresh);
void WS2812_Rotate_Right(uint8_t refresh);
void WS2812_Revvy_Shift_Right(uint8_t refresh);
void WS2812_Revvy_Shift_Left(uint8_t refresh);
